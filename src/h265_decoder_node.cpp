/// @file h265_decoder_node.cpp
/// ROS 2 node that decodes ModalAI VOXL H.265 CompressedImage topics into
/// raw sensor_msgs/Image using FFmpeg (libavcodec) directly.
///
/// Why not GStreamer?  For a single-stream VIO pipeline the appsrc→pipeline→
/// appsink pattern adds unnecessary latency (inter-element buffering, thread
/// hand-offs) and memory copies.  Direct libavcodec gives us:
///   - Synchronous decode in the subscriber callback — zero pipeline latency.
///   - mono8 output = Y-plane copy only — no videoconvert.
///   - Hardware decode via hevc_v4l2m2m (Jetson) / hevc_cuvid (NVIDIA) /
///     generic hevc + VA-API (AMD/Intel) with the same send/receive API.
///   - FFmpeg naturally produces no output when the reference frame is missing,
///     so corrupt-GOP handling is built in with no extra logic.

#include <atomic>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

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
    min_frame_sharpness_ = declare_parameter<double>("min_frame_sharpness", 0.0);
    const auto output_reliability = declare_parameter<std::string>(
      "output_reliability", "reliable");
    const auto output_depth = declare_parameter<int>("output_depth", 30);

    if (output_encoding_ != "bgr8" && output_encoding_ != "mono8") {
      throw std::runtime_error("output_encoding must be bgr8 or mono8");
    }
    if (output_depth <= 0) {
      throw std::runtime_error("output_depth must be greater than zero");
    }
    output_mono_ = output_encoding_ == "mono8";

    // Fallback VPS/SPS/PPS for bags recorded after the encoder's initial
    // parameter packets were already consumed by another subscriber.
    const auto fallback_hex = declare_parameter<std::string>(
      "fallback_codec_params",
      "0000000140010c01ffff016000000300b00000030000030096ac09"
      "00000001420101016000000300b00000030000030096a002808032165aee4c92ea5005da1425"
      "000000014401c0e30f09418f610800");
    fallback_codec_params_ = hexToBytes(fallback_hex);

    // ---- FFmpeg decoder setup ----
    openDecoder(requested_decoder);

    frame_ = av_frame_alloc();
    sw_frame_ = av_frame_alloc();
    pkt_ = av_packet_alloc();
    if (frame_ == nullptr || sw_frame_ == nullptr || pkt_ == nullptr) {
      throw std::runtime_error("Failed to allocate AVFrame/AVPacket");
    }

    // ---- ROS plumbing ----
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

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    input_qos.reliable().durability_volatile();
    encoded_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic, input_qos,
      std::bind(&H265DecoderNode::encodedCallback, this, std::placeholders::_1));

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&H265DecoderNode::logStats, this));

    RCLCPP_INFO(
      get_logger(),
      "Decoding %s (%s) → %s as %s (%s, depth %ld)",
      input_topic.c_str(), decoder_label_.c_str(),
      output_topic.c_str(), output_encoding_.c_str(),
      output_reliability.c_str(), output_depth);
  }

  ~H265DecoderNode() override
  {
    if (sws_ctx_ != nullptr) {
      sws_freeContext(sws_ctx_);
    }
    if (sw_frame_ != nullptr) {
      av_frame_free(&sw_frame_);
    }
    if (frame_ != nullptr) {
      av_frame_free(&frame_);
    }
    if (pkt_ != nullptr) {
      av_packet_free(&pkt_);
    }
    if (codec_ctx_ != nullptr) {
      avcodec_free_context(&codec_ctx_);
    }
    if (hw_device_ctx_ != nullptr) {
      av_buffer_unref(&hw_device_ctx_);
    }
  }

private:
  // ---------------------------------------------------------------------------
  // Decoder setup
  // ---------------------------------------------------------------------------

  /// Allocate a codec context with our strict error-handling settings.
  AVCodecContext * makeContext(const AVCodec * codec)
  {
    AVCodecContext * ctx = avcodec_alloc_context3(codec);
    if (ctx == nullptr) {
      return nullptr;
    }
    // Strict error handling: abort on bitstream errors rather than silently
    // outputting error-concealed garbage (green blocks / stale-reference
    // frames).  Without this, FFmpeg fills in missing data from whatever
    // reference it has — the frame looks "decoded" but is visually wrong,
    // and no flag is set.
    ctx->err_recognition = AV_EF_CRCCHECK;
    ctx->error_concealment = 0;
    return ctx;
  }

  /// Try to open a named HW decoder that requires a specific device context
  /// (e.g. hevc_cuvid needs CUDA, hevc_vaapi needs VAAPI).  If the device
  /// context cannot be created, avcodec_open2 is never called — some FFmpeg
  /// builds segfault when a HW codec is opened without its device.
  bool tryOpenHwDecoder(
    const char * name, AVHWDeviceType device_type,
    const char * device_path = nullptr)
  {
    const AVCodec * codec = avcodec_find_decoder_by_name(name);
    if (codec == nullptr) {
      return false;
    }
    AVCodecContext * ctx = makeContext(codec);
    if (ctx == nullptr) {
      return false;
    }
    AVBufferRef * hw_ctx = nullptr;
    // Try with explicit device path first, then auto-detect.
    if (device_path != nullptr &&
        av_hwdevice_ctx_create(&hw_ctx, device_type, device_path, nullptr, 0) >= 0) {
      // ok
    } else if (av_hwdevice_ctx_create(&hw_ctx, device_type, nullptr, nullptr, 0) < 0) {
      avcodec_free_context(&ctx);
      return false;
    }
    ctx->hw_device_ctx = av_buffer_ref(hw_ctx);
    if (avcodec_open2(ctx, codec, nullptr) < 0) {
      avcodec_free_context(&ctx);
      av_buffer_unref(&hw_ctx);
      return false;
    }
    codec_ctx_ = ctx;
    hw_device_ctx_ = hw_ctx;
    RCLCPP_INFO(
      get_logger(), "Created %s HW device context for %s",
      av_hwdevice_get_type_name(device_type), name);
    return true;
  }

  /// Try to open a decoder without any HW device context.  Works for V4L2
  /// M2M (which opens /dev/video* directly) and for pure software decode.
  bool tryOpenSimpleDecoder(const char * name)
  {
    const AVCodec * codec = avcodec_find_decoder_by_name(name);
    if (codec == nullptr) {
      return false;
    }
    AVCodecContext * ctx = makeContext(codec);
    if (ctx == nullptr) {
      return false;
    }
    if (avcodec_open2(ctx, codec, nullptr) < 0) {
      avcodec_free_context(&ctx);
      return false;
    }
    codec_ctx_ = ctx;
    hw_device_ctx_ = nullptr;
    return true;
  }

  /// get_format callback — when the decoder offers both a HW pixel format
  /// and software formats, select the HW one so decoding stays on the GPU.
  static AVPixelFormat getHwFormat(
    AVCodecContext * ctx, const AVPixelFormat * pix_fmts)
  {
    auto * self = static_cast<H265DecoderNode *>(ctx->opaque);
    for (const AVPixelFormat * p = pix_fmts; *p != AV_PIX_FMT_NONE; ++p) {
      if (*p == self->hw_pix_fmt_) {
        return *p;
      }
    }
    return pix_fmts[0];  // fallback to first (software) format
  }

  /// Open the generic HEVC decoder with hardware acceleration via a HW
  /// device context.  This is how VA-API decoding works in FFmpeg — there
  /// is no separate "hevc_vaapi" decoder; the generic "hevc" decoder
  /// delegates to hardware when a VAAPI (or CUDA/VDPAU) device context is
  /// provided and a get_format callback selects the HW pixel format.
  bool tryOpenGenericHwAccel(
    AVHWDeviceType device_type, const char * device_path = nullptr)
  {
    const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (codec == nullptr) {
      return false;
    }

    // Check that the generic decoder supports this HW device type.
    AVPixelFormat target_fmt = AV_PIX_FMT_NONE;
    for (int i = 0;; ++i) {
      const AVCodecHWConfig * config = avcodec_get_hw_config(codec, i);
      if (config == nullptr) {
        break;
      }
      if (config->device_type == device_type &&
          (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX)) {
        target_fmt = config->pix_fmt;
        break;
      }
    }
    if (target_fmt == AV_PIX_FMT_NONE) {
      return false;  // FFmpeg build doesn't support this HW type for HEVC
    }

    // Create the HW device context.
    AVBufferRef * hw_ctx = nullptr;
    if (device_path != nullptr &&
        av_hwdevice_ctx_create(&hw_ctx, device_type, device_path,
                               nullptr, 0) >= 0) {
      // ok — explicit render node
    } else if (av_hwdevice_ctx_create(&hw_ctx, device_type, nullptr,
                                      nullptr, 0) < 0) {
      return false;
    }

    AVCodecContext * ctx = makeContext(codec);
    if (ctx == nullptr) {
      av_buffer_unref(&hw_ctx);
      return false;
    }
    ctx->hw_device_ctx = av_buffer_ref(hw_ctx);

    // Wire up the get_format callback so the decoder selects the HW format.
    hw_pix_fmt_ = target_fmt;
    ctx->opaque = this;
    ctx->get_format = getHwFormat;

    if (avcodec_open2(ctx, codec, nullptr) < 0) {
      avcodec_free_context(&ctx);
      av_buffer_unref(&hw_ctx);
      hw_pix_fmt_ = AV_PIX_FMT_NONE;
      return false;
    }

    codec_ctx_ = ctx;
    hw_device_ctx_ = hw_ctx;
    RCLCPP_INFO(
      get_logger(), "Opened generic HEVC decoder with %s HW acceleration",
      av_hwdevice_get_type_name(device_type));
    return true;
  }

  /// Open the HEVC decoder.  "auto" tries hardware candidates in order,
  /// falling back to software.
  void openDecoder(const std::string & requested)
  {
    if (requested != "auto") {
      // User-specified: try as a simple (no-context) open first.
      if (tryOpenSimpleDecoder(requested.c_str())) {
        decoder_label_ = requested;
        return;
      }
      throw std::runtime_error("Failed to open decoder: " + requested);
    }

    // Auto-detect in priority order.
    // hevc_v4l2m2m — Jetson NVDEC via V4L2 M2M (no HW device ctx needed)
    if (tryOpenSimpleDecoder("hevc_v4l2m2m")) {
      decoder_label_ = "hevc_v4l2m2m/hardware";
      RCLCPP_INFO(get_logger(), "Auto-selected hardware decoder: hevc_v4l2m2m");
      return;
    }
    // hevc_cuvid — NVIDIA desktop via CUDA (needs CUDA device ctx)
    if (tryOpenHwDecoder("hevc_cuvid", AV_HWDEVICE_TYPE_CUDA)) {
      decoder_label_ = "hevc_cuvid/hardware";
      RCLCPP_INFO(get_logger(), "Auto-selected hardware decoder: hevc_cuvid");
      return;
    }
    // VA-API — Intel / AMD via generic HEVC decoder + HW acceleration.
    // There is no separate "hevc_vaapi" decoder in FFmpeg; VA-API decoding
    // uses the generic "hevc" decoder with a VAAPI device context.
    // In headless / container environments, pass the DRM render node
    // explicitly since there is no display for auto-detect.
    if (tryOpenGenericHwAccel(AV_HWDEVICE_TYPE_VAAPI, "/dev/dri/renderD128")) {
      decoder_label_ = "hevc+vaapi/hardware";
      RCLCPP_INFO(get_logger(), "Auto-selected hardware decoder: hevc + VA-API");
      return;
    }
    // Software fallback — no HW probing (the generic hevc decoder advertises
    // optional HW configs that can crash when the devices aren't present).
    const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (codec == nullptr) {
      throw std::runtime_error("No usable HEVC decoder found");
    }
    AVCodecContext * ctx = makeContext(codec);
    if (ctx == nullptr || avcodec_open2(ctx, codec, nullptr) < 0) {
      if (ctx != nullptr) {
        avcodec_free_context(&ctx);
      }
      throw std::runtime_error("Failed to open software HEVC decoder");
    }
    codec_ctx_ = ctx;
    hw_device_ctx_ = nullptr;
    decoder_label_ = "hevc/software";
    RCLCPP_INFO(get_logger(), "Using software HEVC decoder");
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  /// Scan an Annex B byte-stream and return a bitmask:
  ///   bit 0 — contains VPS/SPS/PPS (NAL types 32-34)
  ///   bit 1 — contains a keyframe (IDR 19-20, CRA 21, BLA 16-18)
  enum NalFlags : unsigned {
    kHasParamSets = 1u << 0,
    kHasKeyframe  = 1u << 1,
  };

  static unsigned scanNalTypes(const uint8_t * data, size_t size)
  {
    unsigned flags = 0;
    for (size_t i = 0; i + 4 < size; ++i) {
      if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1) {
        const uint8_t nal_type = (data[i + 4] >> 1) & 0x3F;
        if (nal_type >= 32 && nal_type <= 34) {
          flags |= kHasParamSets;
        }
        if (nal_type >= 16 && nal_type <= 21) {
          flags |= kHasKeyframe;
        }
        i += 3;
      }
    }
    return flags;
  }

  static bool dataContainsParamSets(const uint8_t * data, size_t size)
  {
    return (scanNalTypes(data, size) & kHasParamSets) != 0;
  }

  /// Compute the mean absolute Laplacian over a subsampled grid.  Measures
  /// how much edge / texture content the frame has.  Real scenes produce
  /// values >> 2 (edges, noise); corrupted gradient / all-black frames
  /// produce ≈ 0 because they are perfectly smooth.
  static double computeSubsampledSharpness(
    const uint8_t * data, int width, int height, int stride)
  {
    constexpr int kStep = 32;
    double sum = 0.0;
    size_t n = 0;
    for (int y = kStep; y < height - kStep; y += kStep) {
      const uint8_t * row = data + static_cast<size_t>(y) * stride;
      const uint8_t * above = data + static_cast<size_t>(y - 1) * stride;
      const uint8_t * below = data + static_cast<size_t>(y + 1) * stride;
      for (int x = kStep; x < width - kStep; x += kStep) {
        // Discrete Laplacian: |4·center − left − right − up − down|
        const int lap = std::abs(
          4 * static_cast<int>(row[x]) -
          static_cast<int>(row[x - 1]) - static_cast<int>(row[x + 1]) -
          static_cast<int>(above[x]) - static_cast<int>(below[x]));
        sum += lap;
        ++n;
      }
    }
    return n > 0 ? sum / static_cast<double>(n) : 0.0;
  }

  static std::vector<uint8_t> hexToBytes(const std::string & hex)
  {
    std::vector<uint8_t> bytes;
    bytes.reserve(hex.size() / 2);
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
      bytes.push_back(
        static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    return bytes;
  }

  /// If the decoded frame lives in hardware memory (e.g. CUDA, VA-API),
  /// transfer it to system memory in sw_frame_ and return a pointer to that.
  /// Otherwise return frame_ as-is.
  AVFrame * ensureSwFrame()
  {
    if (frame_->hw_frames_ctx == nullptr) {
      return frame_;
    }
    av_frame_unref(sw_frame_);
    if (av_hwframe_transfer_data(sw_frame_, frame_, 0) < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Failed to transfer HW frame to system memory");
      return nullptr;
    }
    sw_frame_->pts = frame_->pts;
    return sw_frame_;
  }

  // ---------------------------------------------------------------------------
  // Decode + publish
  // ---------------------------------------------------------------------------

  void encodedCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    ++received_count_;
    if (msg->data.empty()) {
      return;
    }
    if (msg->format != "h265" && msg->format != "hevc") {
      RCLCPP_WARN_ONCE(
        get_logger(), "Expected h265/hevc but input format is '%s'", msg->format.c_str());
    }

    const uint8_t * payload = msg->data.data();
    size_t payload_size = msg->data.size();
    std::vector<uint8_t> augmented;  // only allocated when we prepend params

    // First packet: inject fallback codec params if the stream is missing them.
    if (packet_count_ == 0 && !fallback_codec_params_.empty() &&
        !dataContainsParamSets(payload, payload_size)) {
      augmented.reserve(fallback_codec_params_.size() + payload_size);
      augmented.insert(augmented.end(),
        fallback_codec_params_.begin(), fallback_codec_params_.end());
      augmented.insert(augmented.end(), payload, payload + payload_size);
      payload = augmented.data();
      payload_size = augmented.size();
      RCLCPP_WARN(
        get_logger(),
        "Stream missing VPS/SPS/PPS — prepended %zu-byte fallback codec params",
        fallback_codec_params_.size());
    }
    ++packet_count_;

    const unsigned nal_flags = scanNalTypes(payload, payload_size);
    const bool is_bootstrap = packet_count_ == 1;

    // Flush the DPB at every keyframe so each GOP starts clean and P-frames
    // never decode against a stale reference from the previous GOP.
    if (!is_bootstrap && (nal_flags & kHasKeyframe)) {
      avcodec_flush_buffers(codec_ctx_);
    }

    pkt_->data = const_cast<uint8_t *>(payload);
    pkt_->size = static_cast<int>(payload_size);
    int ret = avcodec_send_packet(codec_ctx_, pkt_);
    if (ret < 0) {
      ++error_count_;
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "avcodec_send_packet failed (%d)", ret);
      return;
    }

    while ((ret = avcodec_receive_frame(codec_ctx_, frame_)) == 0) {
      ++decoded_count_;

      // Transfer from GPU memory if needed (hevc_cuvid, hevc_vaapi).
      AVFrame * out = ensureSwFrame();
      if (out == nullptr) {
        av_frame_unref(frame_);
        continue;
      }

      if (is_bootstrap) {
        RCLCPP_INFO(
          get_logger(), "Dropped H.265 bootstrap frame (%dx%d, %s)",
          out->width, out->height,
          av_get_pix_fmt_name(static_cast<AVPixelFormat>(out->format)));
        av_frame_unref(frame_);
        continue;
      }

      // --- Corruption detection ---
      bool corrupt = (out->flags & AV_FRAME_FLAG_CORRUPT) != 0;
      if (!corrupt && out->decode_error_flags != 0) {
        corrupt = true;
      }
      // Layer 3: Laplacian sharpness — catches gradient / all-black frames
      //          that decode "successfully" but contain no real scene content.
      if (!corrupt && min_frame_sharpness_ > 0.0 && out->data[0] != nullptr) {
        const double sharpness = computeSubsampledSharpness(
          out->data[0], out->width, out->height, out->linesize[0]);
        if (sharpness < min_frame_sharpness_) {
          corrupt = true;
          RCLCPP_DEBUG(
            get_logger(),
            "Frame sharpness %.2f below threshold %.2f — treating as corrupt",
            sharpness, min_frame_sharpness_);
        }
      }
      if (corrupt) {
        ++corrupt_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Skipping corrupt H.265 frame (total: %" PRIu64 ")",
          corrupt_count_.load());
        av_frame_unref(frame_);
        continue;
      }

      publishFrame(out, msg->header.stamp);
      ++published_count_;
      av_frame_unref(frame_);
    }

    if (ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "avcodec_receive_frame returned %d", ret);
    }
  }

  void publishFrame(
    const AVFrame * out, const builtin_interfaces::msg::Time & source_stamp)
  {
    sensor_msgs::msg::Image image;
    image.header.stamp = source_stamp;
    image.header.frame_id = frame_id_;
    image.width = static_cast<uint32_t>(out->width);
    image.height = static_cast<uint32_t>(out->height);
    image.encoding = output_encoding_;
    image.is_bigendian = false;

    const int w = out->width;
    const int h = out->height;

    if (output_mono_) {
      // The Y (luma) plane IS the grayscale image — no conversion.
      image.step = static_cast<uint32_t>(w);
      image.data.resize(static_cast<size_t>(w) * h);
      const uint8_t * src = out->data[0];
      const int src_stride = out->linesize[0];
      if (src_stride == w) {
        std::memcpy(image.data.data(), src, image.data.size());
      } else {
        for (int y = 0; y < h; ++y) {
          std::memcpy(
            image.data.data() + static_cast<size_t>(y) * w,
            src + static_cast<size_t>(y) * src_stride,
            w);
        }
      }
    } else {
      // BGR conversion via swscale (lazy-initialised on first frame).
      const auto src_fmt = static_cast<AVPixelFormat>(out->format);
      if (sws_ctx_ == nullptr || sws_src_fmt_ != src_fmt ||
          sws_w_ != w || sws_h_ != h) {
        if (sws_ctx_ != nullptr) {
          sws_freeContext(sws_ctx_);
        }
        sws_ctx_ = sws_getContext(
          w, h, src_fmt, w, h, AV_PIX_FMT_BGR24,
          SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
        if (sws_ctx_ == nullptr) {
          RCLCPP_ERROR(get_logger(), "Failed to create swscale context");
          return;
        }
        sws_src_fmt_ = src_fmt;
        sws_w_ = w;
        sws_h_ = h;
      }
      const int dst_stride = w * 3;
      image.step = static_cast<uint32_t>(dst_stride);
      image.data.resize(static_cast<size_t>(dst_stride) * h);
      uint8_t * dst_data[1] = {image.data.data()};
      int dst_linesize[1] = {dst_stride};
      sws_scale(
        sws_ctx_, out->data, out->linesize, 0, h, dst_data, dst_linesize);
    }

    decoded_pub_->publish(std::move(image));
  }

  void logStats()
  {
    // Plain RCLCPP_INFO — the wall timer already fires every 5 s, and
    // RCLCPP_INFO_THROTTLE uses the ROS clock which may not advance
    // during bag playback without use_sim_time.
    RCLCPP_INFO(
      get_logger(),
      "H.265 [%s]: received=%" PRIu64 " decoded=%" PRIu64
      " published=%" PRIu64 " corrupt=%" PRIu64
      " errors=%" PRIu64,
      decoder_label_.c_str(),
      received_count_.load(), decoded_count_.load(),
      published_count_.load(), corrupt_count_.load(),
      error_count_.load());
  }

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------

  // FFmpeg
  AVCodecContext * codec_ctx_{nullptr};
  AVBufferRef * hw_device_ctx_{nullptr};
  AVPixelFormat hw_pix_fmt_{AV_PIX_FMT_NONE};  // HW surface format for get_format
  AVFrame * frame_{nullptr};     // raw decode output (may be HW surface)
  AVFrame * sw_frame_{nullptr};  // CPU-side frame after HW transfer
  AVPacket * pkt_{nullptr};
  SwsContext * sws_ctx_{nullptr};
  AVPixelFormat sws_src_fmt_{AV_PIX_FMT_NONE};
  int sws_w_{0};
  int sws_h_{0};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr encoded_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr decoded_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Config
  std::string frame_id_;
  std::string output_encoding_;
  std::string decoder_label_;
  bool output_mono_{false};
  double min_frame_sharpness_{0.0};
  std::vector<uint8_t> fallback_codec_params_;

  // Counters
  uint64_t packet_count_{0};
  std::atomic<uint64_t> received_count_{0};
  std::atomic<uint64_t> decoded_count_{0};
  std::atomic<uint64_t> published_count_{0};
  std::atomic<uint64_t> corrupt_count_{0};
  std::atomic<uint64_t> error_count_{0};
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
