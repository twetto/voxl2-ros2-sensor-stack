#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class H265DecoderNode : public rclcpp::Node
{
public:
  H265DecoderNode()
  : Node("voxl_h265_decoder")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/tracking_front_misp_encoded");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/tracking_front/decoded");
    frame_id_ = declare_parameter<std::string>("frame_id", "tracking_front");
    const auto decoder = declare_parameter<std::string>("decoder", "vah265dec");

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(30));
    output_qos.reliable().durability_volatile();
    decoded_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, output_qos);

    gst_init(nullptr, nullptr);
    const std::string pipeline_description =
      "appsrc name=source is-live=true format=time do-timestamp=true block=false "
      "caps=video/x-h265,stream-format=byte-stream,alignment=au ! "
      "h265parse ! " + decoder + " ! "
      "video/x-raw(memory:SystemMemory),format=NV12 ! "
      "videoconvert ! video/x-raw,format=BGR ! "
      "appsink name=sink emit-signals=true sync=false max-buffers=4 drop=true";

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description.c_str(), &error);
    if (error != nullptr) {
      const std::string message = error->message;
      g_error_free(error);
      throw std::runtime_error("Failed to create GStreamer pipeline: " + message);
    }
    if (pipeline_ == nullptr) {
      throw std::runtime_error("GStreamer returned an empty pipeline");
    }

    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "source");
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (appsrc_ == nullptr || appsink_ == nullptr) {
      throw std::runtime_error("Could not retrieve GStreamer appsrc/appsink");
    }

    GstAppSinkCallbacks callbacks{};
    callbacks.new_sample = &H265DecoderNode::newSampleCallback;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("Failed to start GStreamer hardware decoder pipeline");
    }

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    input_qos.reliable().durability_volatile();
    encoded_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic, input_qos,
      std::bind(&H265DecoderNode::encodedCallback, this, std::placeholders::_1));

    bus_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), std::bind(&H265DecoderNode::pollBus, this));

    RCLCPP_INFO(
      get_logger(), "Hardware-decoding %s with %s and publishing %s",
      input_topic.c_str(), decoder.c_str(), output_topic.c_str());
  }

  ~H265DecoderNode() override
  {
    if (appsrc_ != nullptr) {
      gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
    }
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsink_ != nullptr) {
      gst_object_unref(appsink_);
    }
    if (appsrc_ != nullptr) {
      gst_object_unref(appsrc_);
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
    }
  }

private:
  static GstFlowReturn newSampleCallback(GstAppSink * sink, gpointer user_data)
  {
    return static_cast<H265DecoderNode *>(user_data)->publishSample(sink);
  }

  GstFlowReturn publishSample(GstAppSink * sink)
  {
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr) {
      return GST_FLOW_ERROR;
    }

    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstVideoInfo video_info{};
    GstMapInfo map{};
    const bool valid =
      caps != nullptr && buffer != nullptr &&
      gst_video_info_from_caps(&video_info, caps) &&
      gst_buffer_map(buffer, &map, GST_MAP_READ);

    if (!valid) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    const auto width = GST_VIDEO_INFO_WIDTH(&video_info);
    const auto height = GST_VIDEO_INFO_HEIGHT(&video_info);
    const auto stride = static_cast<uint32_t>(GST_VIDEO_INFO_PLANE_STRIDE(&video_info, 0));
    const auto expected_size = static_cast<size_t>(stride) * height;

    sensor_msgs::msg::Image image;
    image.header.stamp = now();
    image.header.frame_id = frame_id_;
    image.height = height;
    image.width = width;
    image.encoding = "bgr8";
    image.is_bigendian = false;
    image.step = stride;
    image.data.resize(expected_size);
    std::memcpy(image.data.data(), map.data, std::min(expected_size, map.size));

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    decoded_pub_->publish(std::move(image));
    return GST_FLOW_OK;
  }

  void encodedCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }
    if (msg->format != "h265" && msg->format != "hevc") {
      RCLCPP_WARN_ONCE(
        get_logger(), "Expected h265/hevc but input format is '%s'", msg->format.c_str());
    }

    GstBuffer * buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    if (buffer == nullptr) {
      RCLCPP_ERROR(get_logger(), "Failed to allocate an H.265 input buffer");
      return;
    }
    gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
    GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(packet_number_, GST_SECOND, 30);
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, 30);
    ++packet_number_;

    const auto status = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (status != GST_FLOW_OK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "GStreamer rejected an H.265 packet (status %d)", status);
    }
  }

  void pollBus()
  {
    GstBus * bus = gst_element_get_bus(pipeline_);
    while (GstMessage * message = gst_bus_pop_filtered(
        bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING)))
    {
      GError * error = nullptr;
      gchar * details = nullptr;
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
        gst_message_parse_error(message, &error, &details);
        RCLCPP_ERROR(get_logger(), "GStreamer: %s", error->message);
      } else {
        gst_message_parse_warning(message, &error, &details);
        RCLCPP_WARN(get_logger(), "GStreamer: %s", error->message);
      }
      g_clear_error(&error);
      g_free(details);
      gst_message_unref(message);
    }
    gst_object_unref(bus);
  }

  GstElement * pipeline_{nullptr};
  GstElement * appsrc_{nullptr};
  GstElement * appsink_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr encoded_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr decoded_pub_;
  rclcpp::TimerBase::SharedPtr bus_timer_;
  std::string frame_id_;
  uint64_t packet_number_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<H265DecoderNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("voxl_h265_decoder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
