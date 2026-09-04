#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

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
    const auto requested_decoder = declare_parameter<std::string>("decoder", "auto");
    output_encoding_ = declare_parameter<std::string>("output_encoding", "bgr8");
    const auto output_reliability = declare_parameter<std::string>(
      "output_reliability", "reliable");
    const auto output_depth = declare_parameter<int>("output_depth", 30);

    if (output_encoding_ != "bgr8" && output_encoding_ != "mono8") {
      throw std::runtime_error("output_encoding must be bgr8 or mono8");
    }
    if (output_depth <= 0) {
      throw std::runtime_error("output_depth must be greater than zero");
    }

    // Fallback VPS/SPS/PPS for bags recorded after the encoder's initial
    // parameter packets were already consumed by another subscriber.  The
    // default is the VOXL2 tracking-front encoder's current configuration.
    // Update this hex string if you change encoder resolution, tile layout,
    // profile, or bit depth — pure bitrate changes do not affect it.
    const auto fallback_hex = declare_parameter<std::string>(
      "fallback_codec_params",
      "0000000140010c01ffff016000000300b00000030000030096ac09"
      "00000001420101016000000300b00000030000030096a002808032165aee4c92ea5005da1425"
      "000000014401c0e30f09418f610800");
    fallback_codec_params_ = hexToBytes(fallback_hex);

    gst_init(nullptr, nullptr);
    const auto element_available = [](const std::string & name) {
        GstElementFactory * factory = gst_element_factory_find(name.c_str());
        if (factory == nullptr) {
          return false;
        }
        gst_object_unref(factory);
        return true;
      };

    std::string decoder = requested_decoder;
    if (decoder == "auto") {
      if (element_available("nvv4l2decoder") && element_available("nvvidconv")) {
        decoder = "nvv4l2decoder";
      } else if (element_available("vah265dec")) {
        decoder = "vah265dec";
      } else if (element_available("avdec_h265")) {
        decoder = "avdec_h265";
      } else {
        throw std::runtime_error(
                "No supported H.265 decoder found (tried nvv4l2decoder, vah265dec, avdec_h265)");
      }
    } else if (!element_available(decoder)) {
      throw std::runtime_error("Requested GStreamer decoder is unavailable: " + decoder);
    }

    const bool use_jetson_hardware = decoder == "nvv4l2decoder";
    const bool use_hardware = use_jetson_hardware || decoder == "vah265dec";
    if (use_jetson_hardware && !element_available("nvvidconv")) {
      throw std::runtime_error("Jetson decoder selected but nvvidconv is unavailable");
    }

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(output_depth));
    if (output_reliability == "reliable") {
      output_qos.reliable();
    } else if (output_reliability == "best_effort") {
      output_qos.best_effort();
    } else {
      throw std::runtime_error("output_reliability must be reliable or best_effort");
    }
    output_qos.durability_volatile();
    decoded_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, output_qos);

    const std::string parser_and_decoder = use_jetson_hardware ?
      "h265parse config-interval=-1 disable-passthrough=true ! "
      "video/x-h265,stream-format=byte-stream,alignment=au ! "
      "nvv4l2decoder enable-full-frame=true enable-max-performance=true ! " :
      "h265parse ! " + decoder + " ! ";
    const bool output_mono = output_encoding_ == "mono8";
    const std::string output_conversion = use_jetson_hardware ?
      (output_mono ?
        "nvvidconv ! video/x-raw,format=GRAY8 ! " :
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! ") :
      (output_mono ?
        "videoconvert ! video/x-raw,format=GRAY8 ! " :
        "videoconvert ! video/x-raw,format=BGR ! ");
    const std::string pipeline_description =
      "appsrc name=source is-live=true format=time do-timestamp=true block=false "
      "caps=video/x-h265,stream-format=byte-stream,alignment=au ! "
      + parser_and_decoder + output_conversion +
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
      throw std::runtime_error("Failed to start GStreamer decoder pipeline");
    }

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    input_qos.reliable().durability_volatile();
    encoded_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic, input_qos,
      std::bind(&H265DecoderNode::encodedCallback, this, std::placeholders::_1));

    bus_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), std::bind(&H265DecoderNode::pollBus, this));

    RCLCPP_INFO(
      get_logger(), "%s-decoding %s with %s and publishing %s as %s (%s, depth %lld)",
      use_hardware ? "Hardware" : "Software",
      input_topic.c_str(), decoder.c_str(), output_topic.c_str(), output_encoding_.c_str(),
      output_reliability.c_str(), static_cast<long long>(output_depth));
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
  /// Check whether an Annex B byte-stream buffer contains at least one
  /// VPS (32), SPS (33), or PPS (34) NAL unit.
  static bool dataContainsParamSets(const uint8_t * data, size_t size)
  {
    for (size_t i = 0; i + 4 < size; ++i) {
      if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1) {
        const uint8_t nal_type = (data[i + 4] >> 1) & 0x3F;
        if (nal_type >= 32 && nal_type <= 34) {
          return true;
        }
        i += 3;  // skip past the start code
      }
    }
    return false;
  }

  /// Decode a hex string (e.g. "0000000140...") to a byte vector.
  static std::vector<uint8_t> hexToBytes(const std::string & hex)
  {
    std::vector<uint8_t> bytes;
    bytes.reserve(hex.size() / 2);
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
      bytes.push_back(static_cast<uint8_t>(
        std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    return bytes;
  }

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
    const GstClockTime pts = buffer != nullptr ? GST_BUFFER_PTS(buffer) : GST_CLOCK_TIME_NONE;
    int64_t source_stamp_ns = 0;
    const bool has_source_stamp = takeSourceStamp(pts, source_stamp_ns);
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
    ++decoded_frame_count_;

    const auto width = GST_VIDEO_INFO_WIDTH(&video_info);
    const auto height = GST_VIDEO_INFO_HEIGHT(&video_info);
    const auto stride = static_cast<uint32_t>(GST_VIDEO_INFO_PLANE_STRIDE(&video_info, 0));
    const auto expected_size = static_cast<size_t>(stride) * height;

    // VOXL2 emits one stale startup frame before the regular 30 Hz timeline.
    // It is needed to initialize the codec, but must not enter VIO.
    if (!first_decoded_frame_dropped_) {
      first_decoded_frame_dropped_ = true;
      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);
      RCLCPP_INFO(get_logger(), "Dropped the H.265 decoder startup frame");
      return GST_FLOW_OK;
    }

    sensor_msgs::msg::Image image;
    if (has_source_stamp) {
      image.header.stamp = rclcpp::Time(source_stamp_ns);
    } else if (GST_CLOCK_TIME_IS_VALID(pts)) {
      if (!output_time_anchored_) {
        output_anchor_ros_ns_ = now().nanoseconds();
        output_anchor_pts_ = pts;
        output_time_anchored_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Anchored decoded camera time at ROS %.9f (source PTS %.9f)",
          static_cast<double>(output_anchor_ros_ns_) / 1.0e9,
          static_cast<double>(output_anchor_pts_) / GST_SECOND);
      }

      if (pts >= output_anchor_pts_) {
        const auto stamp_ns = output_anchor_ros_ns_ +
          static_cast<int64_t>(pts - output_anchor_pts_);
        image.header.stamp = rclcpp::Time(stamp_ns);
      } else {
        RCLCPP_WARN_ONCE(
          get_logger(), "Decoded H.265 PTS moved backwards; using current ROS time");
        image.header.stamp = now();
      }
    } else {
      RCLCPP_WARN_ONCE(
        get_logger(), "Decoded H.265 frame has no valid PTS; using current ROS time");
      image.header.stamp = now();
    }
    image.header.frame_id = frame_id_;
    image.height = height;
    image.width = width;
    image.encoding = output_encoding_;
    image.is_bigendian = false;
    image.step = stride;
    image.data.resize(expected_size);
    std::memcpy(image.data.data(), map.data, std::min(expected_size, map.size));

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    decoded_pub_->publish(std::move(image));
    ++published_frame_count_;
    return GST_FLOW_OK;
  }

  void encodedCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    ++received_packet_count_;
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

    constexpr uint64_t nominal_frame_ns = GST_SECOND / 30;
    const bool stamp_has_seconds = msg->header.stamp.sec > 0;
    const uint64_t full_stamp_ns = stamp_has_seconds ?
      static_cast<uint64_t>(msg->header.stamp.sec) * GST_SECOND +
      msg->header.stamp.nanosec : 0;
    const uint32_t truncated_stamp = msg->header.stamp.nanosec;

    // If the first packet is missing VPS/SPS/PPS (bag recorded after another
    // subscriber consumed the encoder's initial parameter packets), inject
    // the fallback codec parameters as a synthetic bootstrap before
    // processing the current packet normally.
    if (source_packet_count_ == 0 && !fallback_codec_params_.empty() &&
        !dataContainsParamSets(msg->data.data(), msg->data.size())) {
      GstBuffer * param_buf = gst_buffer_new_allocate(
        nullptr, fallback_codec_params_.size(), nullptr);
      if (param_buf != nullptr) {
        gst_buffer_fill(param_buf, 0, fallback_codec_params_.data(),
          fallback_codec_params_.size());
        GST_BUFFER_PTS(param_buf) = 0;
        GST_BUFFER_DTS(param_buf) = 0;
        GST_BUFFER_DURATION(param_buf) = 0;
        const auto status = gst_app_src_push_buffer(
          GST_APP_SRC(appsrc_), param_buf);
        if (status == GST_FLOW_OK) {
          ++pushed_packet_count_;
        }
        ++source_packet_count_;
        input_pts_ns_ = 0;
        RCLCPP_WARN(get_logger(),
          "Stream missing VPS/SPS/PPS — injected %zu-byte fallback codec parameters",
          fallback_codec_params_.size());
      }
      // Fall through: source_packet_count_ is now 1, so this packet
      // enters the re-anchor branch below.
    }

    if (source_packet_count_ == 0) {
      // The first VOXL2 packet is a codec bootstrap packet. Give it PTS zero;
      // its decoded frame is deliberately discarded below.
      input_pts_ns_ = 0;
    } else if (source_packet_count_ == 1) {
      // Re-anchor on the first regular frame. It must have a distinct PTS for
      // nvv4l2decoder, even when the bootstrap timestamp is discontinuous.
      input_pts_ns_ = nominal_frame_ns;
      RCLCPP_INFO(
        get_logger(), "Anchored regular H.265 stream after bootstrap packet");
    } else {
      int64_t delta_ns;
      if (stamp_has_seconds != previous_stamp_has_seconds_) {
        RCLCPP_WARN(
          get_logger(), "Encoded timestamp representation changed; using one nominal interval");
        delta_ns = nominal_frame_ns;
      } else if (stamp_has_seconds) {
        delta_ns = static_cast<int64_t>(full_stamp_ns) -
          static_cast<int64_t>(previous_full_stamp_ns_);
      } else {
        // Interpret subtraction modulo 2^32 as a signed delta. At 30 Hz this
        // handles every 4.295-second wrap without changing frame timing.
        const uint32_t raw_delta = truncated_stamp - previous_truncated_stamp_;
        delta_ns = raw_delta <= 0x7fffffffU ?
          static_cast<int64_t>(raw_delta) :
          static_cast<int64_t>(raw_delta) - (1LL << 32);
      }

      if (delta_ns <= 0) {
        ++dropped_packet_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping duplicate/out-of-order H.265 packet (delta %.3f ms)",
          static_cast<double>(delta_ns) / 1.0e6);
        gst_buffer_unref(buffer);
        return;
      }
      input_pts_ns_ += static_cast<uint64_t>(delta_ns);
    }

    previous_stamp_has_seconds_ = stamp_has_seconds;
    previous_full_stamp_ns_ = full_stamp_ns;
    previous_truncated_stamp_ = truncated_stamp;
    ++source_packet_count_;
    GST_BUFFER_PTS(buffer) = input_pts_ns_;
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
    GST_BUFFER_DURATION(buffer) = nominal_frame_ns;

    if (stamp_has_seconds) {
      std::lock_guard<std::mutex> lock(source_stamps_mutex_);
      source_stamps_[input_pts_ns_] = static_cast<int64_t>(full_stamp_ns);
      constexpr size_t max_pending_source_stamps = 512;
      while (source_stamps_.size() > max_pending_source_stamps) {
        source_stamps_.erase(source_stamps_.begin());
      }
    }

    const auto status = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (status != GST_FLOW_OK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "GStreamer rejected an H.265 packet (status %d)", status);
    } else {
      ++pushed_packet_count_;
    }
  }

  bool takeSourceStamp(GstClockTime pts, int64_t & stamp_ns)
  {
    if (!GST_CLOCK_TIME_IS_VALID(pts)) {
      return false;
    }

    std::lock_guard<std::mutex> lock(source_stamps_mutex_);
    const auto match = source_stamps_.find(pts);
    if (match == source_stamps_.end()) {
      return false;
    }
    stamp_ns = match->second;
    source_stamps_.erase(match);
    return true;
  }

  void pollBus()
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "H.265 totals: received=%" PRIu64 ", pushed=%" PRIu64
      ", decoded=%" PRIu64 ", published=%" PRIu64 ", input_dropped=%" PRIu64,
      received_packet_count_.load(), pushed_packet_count_.load(),
      decoded_frame_count_.load(), published_frame_count_.load(),
      dropped_packet_count_.load());

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
  std::string output_encoding_;
  std::vector<uint8_t> fallback_codec_params_;
  bool previous_stamp_has_seconds_{false};
  uint64_t previous_full_stamp_ns_{0};
  uint32_t previous_truncated_stamp_{0};
  uint64_t input_pts_ns_{0};
  uint64_t source_packet_count_{0};
  std::atomic<uint64_t> received_packet_count_{0};
  std::atomic<uint64_t> pushed_packet_count_{0};
  std::atomic<uint64_t> dropped_packet_count_{0};
  std::atomic<uint64_t> decoded_frame_count_{0};
  std::atomic<uint64_t> published_frame_count_{0};
  bool first_decoded_frame_dropped_{false};
  bool output_time_anchored_{false};
  GstClockTime output_anchor_pts_{GST_CLOCK_TIME_NONE};
  int64_t output_anchor_ros_ns_{0};
  std::mutex source_stamps_mutex_;
  std::map<GstClockTime, int64_t> source_stamps_;
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
