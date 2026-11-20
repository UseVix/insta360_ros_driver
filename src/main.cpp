#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <cstdio>

#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "insta360_ros_driver/msg/camera_preview_info.hpp"
#include "insta360_ros_driver/msg/exposure_data.hpp"
#include "insta360_ros_driver/msg/camera_compressed_stream_data.hpp"
#include "h26x_stream_info.hpp"

// Host monotonic time in milliseconds (microsecond resolution).
static inline double steadyNowMs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count() / 1000.0;
}

// Latency instrumentation for the camera -> OnVideoData path.
// The hot path (record) only takes a mutex and pushes a couple of doubles; all
// statistics are computed and logged from the report timer.
class LatencyMonitor {
public:
    // offset_ms = host_steady_ms - camera_media_ms
    void setClockOffset(double offset_ms, double rtt_ms) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (offset_valid_) {
            offset_drift_ms_ = offset_ms - offset_ms_;
        }
        offset_ms_ = offset_ms;
        sync_rtt_ms_ = rtt_ms;
        offset_valid_ = true;
    }

    void record(int stream_index, int64_t cam_ts_ms, double arrival_ms) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto& s = streams_[stream_index];
        if (offset_valid_) {
            s.latency.push_back(arrival_ms - static_cast<double>(cam_ts_ms) - offset_ms_);
        }
        if (s.has_last) {
            s.host_dt.push_back(arrival_ms - s.last_arrival_ms);
        }
        s.last_arrival_ms = arrival_ms;
        s.has_last = true;
    }

    void report(const rclcpp::Logger& logger) {
        std::map<int, std::pair<std::vector<double>, std::vector<double>>> windows;
        double offset, rtt, drift;
        bool valid;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto& [idx, s] : streams_) {
                auto& w = windows[idx];
                w.first.swap(s.latency);
                w.second.swap(s.host_dt);
            }
            offset = offset_ms_;
            rtt = sync_rtt_ms_;
            drift = offset_drift_ms_;
            valid = offset_valid_;
        }

        if (valid) {
            RCLCPP_INFO(logger, "[latency] clock sync: offset=%.3f ms, sync rtt=%.3f ms (uncertainty ~ +/-%.2f ms), drift since prev sync=%+.3f ms",
                        offset, rtt, rtt / 2.0 + 1.0, drift);
        } else {
            RCLCPP_WARN(logger, "[latency] no valid clock sync yet -> absolute latency unavailable");
        }

        for (auto& [idx, w] : windows) {
            const Summary lat = summarize(w.first);
            const Summary dt = summarize(w.second);
            RCLCPP_INFO(logger,
                "[latency] stream %d | cam->callback ms: min=%.2f mean=%.2f p50=%.2f p99=%.2f max=%.2f | "
                "inter-arrival ms: min=%.2f mean=%.2f p50=%.2f p99=%.2f max=%.2f",
                idx, lat.min, lat.mean, lat.p50, lat.p99, lat.max,
                dt.min, dt.mean, dt.p50, dt.p99, dt.max);
        }
    }

private:
    struct StreamState {
        std::vector<double> latency, host_dt;
        double last_arrival_ms = 0.0;
        bool has_last = false;
    };
    struct Summary { double min = NAN, mean = NAN, p50 = NAN, p99 = NAN, max = NAN; };

    static Summary summarize(std::vector<double>& v) {
        Summary s;
        if (v.empty()) return s;
        std::sort(v.begin(), v.end());
        double sum = 0.0;
        for (double x : v) sum += x;
        auto pct = [&](double p) {
            size_t i = static_cast<size_t>(std::ceil(p * v.size()));
            return v[std::min(v.size() - 1, i == 0 ? 0 : i - 1)];
        };
        s.min = v.front();
        s.max = v.back();
        s.mean = sum / v.size();
        s.p50 = pct(0.50);
        s.p99 = pct(0.99);
        return s;
    }

    std::mutex mtx_;
    std::map<int, StreamState> streams_;
    double offset_ms_ = 0.0;
    double sync_rtt_ms_ = 0.0;
    double offset_drift_ms_ = 0.0;
    bool offset_valid_ = false;
};

// Checks the incoming stream against the requested live-stream settings.
// Resolution comes from the SPS inside the bitstream, frame rate from the camera timestamps of
// consecutive frames, and bitrate from the bytes received per unit of camera time.
// The SPS resolution is logged once (and again if it changes); fps/bitrate are reported periodically.
class StreamFormatMonitor {
public:
    struct Requested {
        std::string name;  // e.g. "1920x960p30"
        int width = 0;
        int height = 0;
        double fps = 0.0;
        double bitrate = 0.0;  // bit/s
    };

    StreamFormatMonitor(h26x::Codec codec, Requested req, rclcpp::Logger logger, double fps_tol, double bitrate_tol)
        : codec_(codec), req_(std::move(req)), logger_(logger), fps_tol_(fps_tol), bitrate_tol_(bitrate_tol) {}

    void onPacket(int stream_index, const uint8_t* data, size_t size, int64_t cam_ts_ms) {
        bool has_vcl = false, has_key = false, has_sps = false;
        h26x::SpsInfo sps;
        h26x::forEachNal(data, size, [&](const uint8_t* nal, size_t n) {
            const int type = h26x::nalType(codec_, nal, n);
            if (h26x::isVcl(codec_, type)) has_vcl = true;
            if (h26x::isKeyframe(codec_, type)) has_key = true;
            if (h26x::isSps(codec_, type) && !has_sps) has_sps = h26x::parseSps(codec_, nal, n, sps);
        });

        std::lock_guard<std::mutex> lk(mtx_);
        auto& s = streams_[stream_index];
        s.bytes += size;
        s.max_packet_bytes = std::max<uint64_t>(s.max_packet_bytes, size);
        // A frame may span several packets/slices; count distinct camera timestamps instead.
        if (has_vcl && (s.frames == 0 || cam_ts_ms != s.last_ts)) {
            if (s.frames == 0) s.first_ts = cam_ts_ms;
            s.last_ts = cam_ts_ms;
            ++s.frames;
            if (has_key) ++s.keyframes;
        }
        if (has_sps && (sps.width != s.width || sps.height != s.height || sps.fps != s.sps_fps)) {
            const bool changed = s.width != 0;
            s.width = sps.width;
            s.height = sps.height;
            s.sps_fps = sps.fps;
            const char* verdict = resolutionVerdict(s.width, s.height);
            if (changed || std::string(verdict) != "MATCH") {
                RCLCPP_WARN(logger_, "[stream] stream %d: %s SPS %s resolution %dx%d (sps fps %s) | requested %s -> resolution %s",
                            stream_index, codecName(), changed ? "changed to" : "reports", s.width, s.height,
                            fmtFps(s.sps_fps).c_str(), req_.name.c_str(), verdict);
            } else {
                RCLCPP_INFO(logger_, "[stream] stream %d: %s SPS reports resolution %dx%d (sps fps %s) | requested %s -> resolution %s",
                            stream_index, codecName(), s.width, s.height, fmtFps(s.sps_fps).c_str(), req_.name.c_str(), verdict);
            }
        }
    }

    // Logs one line per stream for the window since the previous report, then starts a new window.
    void report() {
        std::map<int, Window> windows;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto& [idx, s] : streams_) {
                windows[idx] = s;
                s.resetWindow();
            }
        }
        if (windows.empty()) {
            RCLCPP_WARN(logger_, "[stream] no video packets received in this window");
        }
        for (const auto& [idx, w] : windows) {
            double fps = NAN, bitrate = NAN;
            if (w.frames >= 2 && w.last_ts > w.first_ts) {
                const double span_s = (w.last_ts - w.first_ts) / 1000.0;
                fps = (w.frames - 1) / span_s;
                bitrate = w.bytes * 8.0 / (w.frames / fps);  // duration covered by `frames` frames
            }
            const double ratio = bitrate / req_.bitrate;
            const std::string resolution = w.width ? std::to_string(w.width) + "x" + std::to_string(w.height) : "n/a";
            RCLCPP_INFO(logger_,
                "[stream] stream %d | resolution=%s requested=%dx%d %s | fps=%s requested=%.2f %s | "
                "bitrate=%s requested=%.0f ratio=%s %s | sps_fps=%s frames=%lu keyframes=%lu max_packet_bytes=%lu",
                idx, resolution.c_str(),
                req_.width, req_.height, resolutionVerdict(w.width, w.height),
                fmt(fps, "%.2f").c_str(), req_.fps, toleranceVerdict(fps, req_.fps, fps_tol_),
                fmt(bitrate, "%.0f").c_str(), req_.bitrate, fmt(ratio, "%.2f").c_str(),
                bitrateVerdict(bitrate, req_.bitrate, bitrate_tol_),
                fmtFps(w.sps_fps).c_str(), static_cast<unsigned long>(w.frames),
                static_cast<unsigned long>(w.keyframes), static_cast<unsigned long>(w.max_packet_bytes));
        }
    }

private:
    struct Window {
        // persistent across windows
        int width = 0, height = 0;
        double sps_fps = 0.0;
        // per window
        uint64_t bytes = 0, frames = 0, keyframes = 0, max_packet_bytes = 0;
        int64_t first_ts = 0, last_ts = 0;
        void resetWindow() {
            // Keep last_ts so the first frame of the next window is not mistaken for a duplicate.
            bytes = frames = keyframes = max_packet_bytes = 0;
            first_ts = 0;
        }
    };

    const char* codecName() const { return codec_ == h26x::Codec::H264 ? "h264" : "h265"; }
    const char* resolutionVerdict(int w, int h) const {
        if (w == 0) return "UNKNOWN";
        return (w == req_.width && h == req_.height) ? "MATCH" : "MISMATCH";
    }
    static const char* toleranceVerdict(double actual, double requested, double tol) {
        if (!std::isfinite(actual) || requested <= 0.0) return "UNKNOWN";
        return std::fabs(actual - requested) <= tol * requested ? "MATCH" : "MISMATCH";
    }
    // Encoders undershoot on easy-to-compress scenes, so a low bitrate alone does not prove the
    // request was ignored; ABOVE (well over the request) is the clear sign that it was.
    static const char* bitrateVerdict(double actual, double requested, double tol) {
        if (!std::isfinite(actual) || requested <= 0.0) return "UNKNOWN";
        if (actual > requested * (1.0 + tol)) return "ABOVE";
        if (actual < requested * (1.0 - tol)) return "BELOW";
        return "MATCH";
    }
    static std::string fmt(double v, const char* f) {
        if (!std::isfinite(v)) return "n/a";
        char buf[64];
        std::snprintf(buf, sizeof(buf), f, v);
        return buf;
    }
    static std::string fmtFps(double v) { return v > 0.0 ? fmt(v, "%.2f") : "n/a"; }

    h26x::Codec codec_;
    Requested req_;
    rclcpp::Logger logger_;
    double fps_tol_, bitrate_tol_;
    std::mutex mtx_;
    std::map<int, Window> streams_;
};

class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<insta360_ros_driver::msg::CameraCompressedStreamData>::SharedPtr stream_data_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<insta360_ros_driver::msg::ExposureData>::SharedPtr exposure_pub_;
    std::string video_format_;
    std::shared_ptr<LatencyMonitor> latency_;  // null when measurement is disabled
    std::shared_ptr<StreamFormatMonitor> format_;  // null when stream checking is disabled
    std::atomic<bool> first_frame_logged_{false};
    std::atomic<bool> first_exposure_logged_{false};
    std::atomic<double> last_exposure_time_s_{NAN};

public:
    TestStreamDelegate(const std::shared_ptr<rclcpp::Node>& node, const std::string& video_format = "h264",
                       std::shared_ptr<LatencyMonitor> latency = nullptr,
                       std::shared_ptr<StreamFormatMonitor> format = nullptr)
        : node_(node), video_format_(video_format), latency_(std::move(latency)), format_(std::move(format)) {
        if (!node_->has_parameter("compressed_stream_topic")) {
            node_->declare_parameter("compressed_stream_topic", "/dual_fisheye/image/compressed_stream");
        }
        if (!node_->has_parameter("imu_topic")) {
            node_->declare_parameter("imu_topic", "imu/data_raw");
        }
        if (!node_->has_parameter("exposure_data_topic")) {
            node_->declare_parameter("exposure_data_topic", "/camera/exposure_data");
        }
        std::string stream_topic = node_->get_parameter("compressed_stream_topic").as_string();
        std::string imu_topic = node_->get_parameter("imu_topic").as_string();
        std::string exposure_topic = node_->get_parameter("exposure_data_topic").as_string();

        // Publisher for custom CameraCompressedStreamData including stream_type and stream_index
        stream_data_pub_ = node_->create_publisher<insta360_ros_driver::msg::CameraCompressedStreamData>(
            stream_topic, 
            rclcpp::QoS(10)
        );

        // Publisher for IMU data
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
        
        // Publisher for exposure data
        exposure_pub_ = node_->create_publisher<insta360_ros_driver::msg::ExposureData>(exposure_topic, rclcpp::SystemDefaultsQoS());
        RCLCPP_INFO(node_->get_logger(), "Publisher for compressed stream data (%s) on %s and IMU on %s created.", video_format_.c_str(), stream_topic.c_str(), imu_topic.c_str());
    }

    virtual ~TestStreamDelegate() {}

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {}

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        // Take the arrival time before doing any work so it reflects when the SDK handed us the packet.
        const double arrival_ms = latency_ ? steadyNowMs() : 0.0;

        if (latency_ && !first_frame_logged_.exchange(true)) {
            const int64_t epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            RCLCPP_INFO(node_->get_logger(),
                "[latency] first video packet: camera timestamp=%ld ms, host system time=%ld ms "
                "(if these are far apart the camera timestamp is not Unix epoch time)",
                static_cast<long>(timestamp), static_cast<long>(epoch_ms));
        }

        if (size > 0) {
            // Publish custom message with explicit stream_type and stream_index
            if (stream_data_pub_) {
                auto stream_msg = std::make_unique<insta360_ros_driver::msg::CameraCompressedStreamData>();
                stream_msg->stream_type = streamType;
                stream_msg->stream_index = stream_index;
                stream_msg->data.header.stamp.sec = timestamp / 1000;
                stream_msg->data.header.stamp.nanosec = (timestamp % 1000) * 1000000;
                stream_msg->data.header.frame_id = "camera_frame";
                stream_msg->data.format = video_format_;
                stream_msg->data.data.assign(data, data + size);
                stream_data_pub_->publish(std::move(stream_msg));
            }
            if (latency_) {
                latency_->record(stream_index, timestamp, arrival_ms);
            }
            if (format_) {
                format_->onPacket(stream_index, data, size, timestamp);
            }
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        for (const auto& gyro : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp.sec = gyro.timestamp / 1000;
            msg->header.stamp.nanosec = (gyro.timestamp % 1000) * 1000000;
            msg->header.frame_id = "imu_frame";
            msg->angular_velocity.x = gyro.gx;
            msg->angular_velocity.y = gyro.gy;
            msg->angular_velocity.z = gyro.gz;
            
            msg->linear_acceleration.x = gyro.ax * 9.80665;
            msg->linear_acceleration.y = gyro.ay * 9.80665;
            msg->linear_acceleration.z = gyro.az * 9.80665;

            msg->orientation.x = 0.0;
            msg->orientation.y = 0.0;
            msg->orientation.z = 0.0;
            msg->orientation.w = 1.0; // Neutral orientation
            msg->orientation_covariance[0] = -1.0; // No orientation data available

            for (int i = 0; i < 9; i++)
            {
                msg->angular_velocity_covariance[i] = 0;
                msg->linear_acceleration_covariance[i] = 0;
            }
            imu_pub_->publish(std::move(msg));
        }
    }

    // Most recent exposure time reported by the camera, in seconds (NaN until the first report).
    double lastExposureTime() const { return last_exposure_time_s_.load(); }

    void OnExposureData(const ins_camera::ExposureData& data) override {
        last_exposure_time_s_.store(data.exposure_time);
        if (!first_exposure_logged_.exchange(true)) {
            // Confirms whether the requested shutter speed was actually applied by the camera.
            RCLCPP_INFO(node_->get_logger(), "First exposure report from camera: exposure_time=%.6f s (1/%.0f s)",
                        data.exposure_time, data.exposure_time > 0.0 ? 1.0 / data.exposure_time : 0.0);
        }
        if (exposure_pub_) {
            auto msg = std::make_unique<insta360_ros_driver::msg::ExposureData>();
            msg->timestamp = data.timestamp;
            msg->exposure_time = data.exposure_time;
            exposure_pub_->publish(std::move(msg));
        }
    }
};

class CameraWrapper {
private:
    std::shared_ptr<ins_camera::Camera> cam;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<insta360_ros_driver::msg::CameraPreviewInfo>::SharedPtr metadata_pub_;

    // Latency measurement
    bool measure_latency_ = false;
    int clock_sync_samples_ = 30;
    std::shared_ptr<LatencyMonitor> latency_;
    std::shared_ptr<TestStreamDelegate> delegate_;
    std::shared_ptr<StreamFormatMonitor> format_;
    rclcpp::TimerBase::SharedPtr clock_sync_timer_;
    rclcpp::TimerBase::SharedPtr latency_report_timer_;

    // Live-stream resolutions that can be selected with the `video_resolution` parameter.
    // Not every model/firmware accepts every entry; StartLiveStreaming() fails if it does not.
    static bool parseResolution(const std::string& name, ins_camera::VideoResolution& out) {
        static const std::map<std::string, ins_camera::VideoResolution> table = {
            {"3840x1920p30", ins_camera::VideoResolution::RES_3840_1920P30},
            {"2560x1280p30", ins_camera::VideoResolution::RES_2560_1280P30},
            {"2304x1152p30", ins_camera::VideoResolution::RES_1152_1152P30},
            {"1920x960p30",  ins_camera::VideoResolution::RES_1920_960P30},
            {"1440x720p30",  ins_camera::VideoResolution::RES_1440_720P30},
            {"960x480p30",   ins_camera::VideoResolution::RES_960_480P30},
            {"3840x1920p60", ins_camera::VideoResolution::RES_3840_1920P60},
            {"2560x1280p60", ins_camera::VideoResolution::RES_2560_1280P60},
        };
        auto it = table.find(name);
        if (it == table.end()) return false;
        out = it->second;
        return true;
    }

    static bool parseExposureMode(const std::string& name, ins_camera::PhotographyOptions_ExposureMode& out) {
        static const std::map<std::string, ins_camera::PhotographyOptions_ExposureMode> table = {
            {"auto", ins_camera::PhotographyOptions_ExposureMode::AUTO},
            {"iso_priority", ins_camera::PhotographyOptions_ExposureMode::ISO_PRIORITY},
            {"shutter_priority", ins_camera::PhotographyOptions_ExposureMode::SHUTTER_PRIORITY},
            {"manual", ins_camera::PhotographyOptions_ExposureMode::MANUAL},
            {"adaptive", ins_camera::PhotographyOptions_ExposureMode::ADAPTIVE},
            {"full_auto", ins_camera::PhotographyOptions_ExposureMode::FULL_AUTO},
        };
        auto it = table.find(name);
        if (it == table.end()) return false;
        out = it->second;
        return true;
    }

    // Applies exposure parameters to the live-stream function mode. A short, fixed shutter
    // stops auto-exposure from stretching the exposure (and dropping the frame rate) in low light.
    void applyExposureSettings() {
        const std::string mode_name = declareParam<std::string>("exposure_mode", "shutter_priority");
        const int64_t shutter_den = declareParam<int64_t>("shutter_speed_denominator", 120);
        const int64_t iso = declareParam<int64_t>("iso", 0);
        const int64_t ev_bias = declareParam<int64_t>("ev_bias", 0);
        const int64_t iso_top_limit = declareParam<int64_t>("video_iso_top_limit", 0);

        if (mode_name == "unchanged") {
            RCLCPP_INFO(node_->get_logger(), "exposure_mode=unchanged -> leaving camera exposure settings as they are");
            return;
        }
        ins_camera::PhotographyOptions_ExposureMode mode;
        if (!parseExposureMode(mode_name, mode)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Unknown exposure_mode '%s' (use unchanged|auto|shutter_priority|iso_priority|manual|adaptive|full_auto); "
                "leaving exposure unchanged", mode_name.c_str());
            return;
        }

        auto settings = std::make_shared<ins_camera::ExposureSettings>();
        settings->SetExposureMode(mode);
        settings->SetEVBias(static_cast<int32_t>(ev_bias));
        if (shutter_den > 0) settings->SetShutterSpeed(1.0 / static_cast<double>(shutter_den));
        if (iso > 0) settings->SetIso(static_cast<int32_t>(iso));
        if (iso_top_limit > 0) settings->SetVideoISOTopLimit(static_cast<int32_t>(iso_top_limit));

        const bool ok = cam->SetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM, settings);
        if (!ok) {
            RCLCPP_WARN(node_->get_logger(), "SetExposureSettings(LIVE_STREAM) failed (mode=%s, shutter=1/%ld s, iso=%ld)",
                        mode_name.c_str(), static_cast<long>(shutter_den), static_cast<long>(iso));
            return;
        }
        auto applied = cam->GetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM);
        if (applied) {
            RCLCPP_INFO(node_->get_logger(),
                "Exposure settings applied: requested mode=%s shutter=1/%ld s iso=%ld ev=%ld | camera reports mode=%d shutter=%.6f s iso=%d ev=%d",
                mode_name.c_str(), static_cast<long>(shutter_den), static_cast<long>(iso), static_cast<long>(ev_bias),
                static_cast<int>(applied->ExposureMode()), applied->ShutterSpeed(), applied->Iso(), applied->EVBias());
        } else {
            RCLCPP_INFO(node_->get_logger(), "Exposure settings applied: mode=%s shutter=1/%ld s",
                        mode_name.c_str(), static_cast<long>(shutter_den));
        }
    }

    static bool parseVideoSubMode(const std::string& name, ins_camera::SubVideoMode& out) {
        static const std::map<std::string, ins_camera::SubVideoMode> table = {
            {"normal", ins_camera::SubVideoMode::VIDEO_NORMAL},
            {"bullet_time", ins_camera::SubVideoMode::VIDEO_BULLETTIME},
            {"timelapse", ins_camera::SubVideoMode::VIDEO_TIMELAPSE},
            {"hdr", ins_camera::SubVideoMode::VIDEO_HDR},
            {"timeshift", ins_camera::SubVideoMode::VIDEO_TIMESHIFT},
            {"super", ins_camera::SubVideoMode::VIDEO_SUPER},
            {"loop_recording", ins_camera::SubVideoMode::VIDEO_LOOPRECORDING},
            {"slow_motion", ins_camera::SubVideoMode::VIDEO_SLOW_MOTION},
            {"selfie", ins_camera::SubVideoMode::VIDEO_SELFIE},
            {"pure", ins_camera::SubVideoMode::VIDEO_PURE},          // PureVideo (low light)
            {"liveview", ins_camera::SubVideoMode::VIDEO_LIVEVIEW},  // live-streaming mode (X4/X5)
            {"dash_cam", ins_camera::SubVideoMode::VIDEO_DASH_CAM},
            {"ptz", ins_camera::SubVideoMode::VIDEO_PTZ},
        };
        auto it = table.find(name);
        if (it == table.end()) return false;
        out = it->second;
        return true;
    }

    static bool parseWhiteBalance(const std::string& name, ins_camera::PhotographyOptions_WhiteBalance& out) {
        static const std::map<std::string, ins_camera::PhotographyOptions_WhiteBalance> table = {
            {"auto", ins_camera::PhotographyOptions_WhiteBalance::WB_AUTO},
            {"2700k", ins_camera::PhotographyOptions_WhiteBalance::WB_2700K},
            {"4000k", ins_camera::PhotographyOptions_WhiteBalance::WB_4000K},
            {"5000k", ins_camera::PhotographyOptions_WhiteBalance::WB_5000K},
            {"6500k", ins_camera::PhotographyOptions_WhiteBalance::WB_6500K},
            {"7500k", ins_camera::PhotographyOptions_WhiteBalance::WB_7500K},
        };
        auto it = table.find(name);
        if (it == table.end()) return false;
        out = it->second;
        return true;
    }

    // Selects the camera's video sub-mode (the mode chosen on the camera screen). The SDK has no getter,
    // so the camera keeps whatever was set last; "unchanged" leaves it alone.
    bool applyVideoSubMode() {
        const std::string name = declareParam<std::string>("video_sub_mode", "unchanged");
        if (name == "unchanged") return true;
        ins_camera::SubVideoMode mode;
        if (!parseVideoSubMode(name, mode)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Unknown video_sub_mode '%s' (use unchanged|normal|hdr|pure|liveview|loop_recording|timelapse|timeshift|"
                "bullet_time|slow_motion|selfie|super|dash_cam|ptz)", name.c_str());
            return false;
        }
        if (cam->SetVideoSubMode(mode)) {
            RCLCPP_INFO(node_->get_logger(), "Video sub-mode set: %s", name.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "SetVideoSubMode(%s) failed", name.c_str());
        }
        return true;
    }

    // Contrast / saturation / brightness / sharpness / white balance of the live-stream mode.
    // Only applied when apply_image_settings is true; the defaults are the SDK's documented defaults.
    bool applyImageSettings() {
        const bool apply = declareParam<bool>("apply_image_settings", false);
        const int64_t contrast = declareParam<int64_t>("image_contrast", 64);
        const int64_t saturation = declareParam<int64_t>("image_saturation", 64);
        const int64_t brightness = declareParam<int64_t>("image_brightness", 0);
        const int64_t sharpness = declareParam<int64_t>("image_sharpness", 3);
        const std::string wb_name = declareParam<std::string>("image_white_balance", "auto");
        if (!apply) return true;

        ins_camera::PhotographyOptions_WhiteBalance wb;
        if (!parseWhiteBalance(wb_name, wb)) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown image_white_balance '%s' (use auto|2700k|4000k|5000k|6500k|7500k)",
                         wb_name.c_str());
            return false;
        }
        if (contrast < 0 || contrast > 256 || saturation < 0 || saturation > 256 || brightness < -256 ||
            brightness > 256 || sharpness < 0 || sharpness > 6) {
            RCLCPP_ERROR(node_->get_logger(),
                "Image setting out of range (contrast 0..256, saturation 0..256, brightness -256..256, sharpness 0..6)");
            return false;
        }
        using CS = ins_camera::CaptureSettings;
        auto settings = std::make_shared<CS>();
        settings->SetValue(CS::CaptureSettings_Contrast, static_cast<int32_t>(contrast));
        settings->SetValue(CS::CaptureSettings_Saturation, static_cast<int32_t>(saturation));
        settings->SetValue(CS::CaptureSettings_Brightness, static_cast<int32_t>(brightness));
        settings->SetValue(CS::CaptureSettings_Sharpness, static_cast<int32_t>(sharpness));
        settings->SetWhiteBalance(wb);
        if (!cam->SetCaptureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM, settings)) {
            RCLCPP_WARN(node_->get_logger(),
                "SetCaptureSettings(LIVE_STREAM) failed (contrast=%ld saturation=%ld brightness=%ld sharpness=%ld wb=%s)",
                static_cast<long>(contrast), static_cast<long>(saturation), static_cast<long>(brightness),
                static_cast<long>(sharpness), wb_name.c_str());
            return true;
        }
        auto applied = cam->GetCaptureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM);
        if (applied) {
            RCLCPP_INFO(node_->get_logger(),
                "Image settings applied: requested contrast=%ld saturation=%ld brightness=%ld sharpness=%ld wb=%s | "
                "camera reports contrast=%d saturation=%d brightness=%d sharpness=%d wb=%d",
                static_cast<long>(contrast), static_cast<long>(saturation), static_cast<long>(brightness),
                static_cast<long>(sharpness), wb_name.c_str(),
                applied->GetIntValue(CS::CaptureSettings_Contrast), applied->GetIntValue(CS::CaptureSettings_Saturation),
                applied->GetIntValue(CS::CaptureSettings_Brightness), applied->GetIntValue(CS::CaptureSettings_Sharpness),
                static_cast<int>(applied->WhiteBalance()));
        } else {
            RCLCPP_INFO(node_->get_logger(), "Image settings applied");
        }
        return true;
    }

    template <typename T>
    T declareParam(const std::string& name, const T& default_value) {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<T>(name, default_value);
        }
        return node_->get_parameter(name).get_value<T>();
    }

    // NTP-style offset estimate: query the camera's media clock several times and
    // keep the sample with the smallest round trip, whose midpoint is the most
    // accurate estimate of when the camera read its clock.
    void syncClock() {
        double best_rtt = std::numeric_limits<double>::infinity();
        double best_offset = 0.0;
        int64_t best_cam = 0;
        for (int i = 0; i < clock_sync_samples_; ++i) {
            const double t0 = steadyNowMs();
            const int64_t cam_ms = cam->GetCameraMediaTime();
            const double t1 = steadyNowMs();
            if (cam_ms <= 0) continue;
            const double rtt = t1 - t0;
            if (rtt < best_rtt) {
                best_rtt = rtt;
                best_offset = (t0 + t1) / 2.0 - static_cast<double>(cam_ms);
                best_cam = cam_ms;
            }
        }
        if (!std::isfinite(best_rtt)) {
            RCLCPP_WARN(node_->get_logger(), "[latency] clock sync failed: GetCameraMediaTime() returned no valid samples");
            return;
        }
        latency_->setClockOffset(best_offset, best_rtt);
        RCLCPP_DEBUG(node_->get_logger(), "[latency] clock sync: camera media time=%ld ms, offset=%.3f ms, rtt=%.3f ms",
                     static_cast<long>(best_cam), best_offset, best_rtt);
    }

public:
    CameraWrapper(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

    ~CameraWrapper() {
        if (cam) {
            cam->StopLiveStreaming();
            cam->Close();
        }
    }

    int run_camera() {
        ins_camera::DeviceDiscovery discovery;

        if (!node_->has_parameter("camera_preview_info_topic")) {
            node_->declare_parameter("camera_preview_info_topic", "/insta360/camera_preview_info");
        }
        std::string preview_topic = node_->get_parameter("camera_preview_info_topic").as_string();

        rclcpp::QoS metadata_qos(1);
        metadata_qos.reliable().transient_local();
        metadata_pub_ = node_->create_publisher<insta360_ros_driver::msg::CameraPreviewInfo>(
            preview_topic, metadata_qos);

        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No available camera devices found.");
            return -1;
        }

        cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam->Open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open camera.");
            return -1;
        }
        RCLCPP_INFO(node_->get_logger(), "Camera opened successfully.");

        auto preview = cam->GetPreviewParam();

        auto metadata = std::make_unique<insta360_ros_driver::msg::CameraPreviewInfo>();
        metadata->header.stamp = node_->get_clock()->now();
        metadata->header.frame_id = "camera_frame";
        metadata->camera_name = preview.camera_name;
        metadata->decode_type = preview.encode_type == ins_camera::VideoEncodeType::H265 ? "h265" : "h264";
        metadata->crop_src_width = preview.crop_info.src_width;
        metadata->crop_src_height = preview.crop_info.src_height;
        metadata->crop_dst_width = preview.crop_info.dst_width;
        metadata->crop_dst_height = preview.crop_info.dst_height;
        metadata->crop_offset_x = preview.crop_info.crop_offset_x;
        metadata->crop_offset_y = preview.crop_info.crop_offset_y;
        metadata->camera_offset = preview.offset;
        metadata->delay_timestamp = preview.delay_timestamp;
        metadata->sweep_time = preview.sweep_time;
        metadata_pub_->publish(std::move(metadata));
        discovery.FreeDeviceDescriptors(list);

        std::string video_format = preview.encode_type == ins_camera::VideoEncodeType::H265 ? "h265" : "h264";
        // The codec is chosen on the camera's menu; the SDK can only read it.
        RCLCPP_INFO(node_->get_logger(), "Camera encode type: %s", video_format.c_str());
        measure_latency_ = declareParam<bool>("measure_latency", false);
        const double report_period_s = declareParam<double>("latency_report_period_s", 5.0);
        const double sync_period_s = declareParam<double>("clock_sync_period_s", 10.0);
        clock_sync_samples_ = std::max<int64_t>(1, declareParam<int64_t>("clock_sync_samples", 30));
        if (measure_latency_) {
            latency_ = std::make_shared<LatencyMonitor>();
        }

        const std::string resolution_name = declareParam<std::string>("video_resolution", "1920x960p30");
        const int64_t video_bitrate = declareParam<int64_t>("video_bitrate", 1024 * 1024 / 2);

        ins_camera::LiveStreamParam param;
        if (!parseResolution(resolution_name, param.video_resolution)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Unknown video_resolution '%s' (use 3840x1920p30|2560x1280p30|2304x1152p30|1920x960p30|1440x720p30|960x480p30|"
                "3840x1920p60|2560x1280p60)", resolution_name.c_str());
            return -1;
        }
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = static_cast<uint32_t>(std::max<int64_t>(1, video_bitrate));
        param.enable_audio = false;
        param.using_lrv = false;
        RCLCPP_INFO(node_->get_logger(), "Live stream: resolution=%s bitrate=%u bit/s | camera preview info: %dx%d (crop to %dx%d)",
                    resolution_name.c_str(), param.video_bitrate,
                    static_cast<int>(preview.crop_info.src_width), static_cast<int>(preview.crop_info.src_height),
                    static_cast<int>(preview.crop_info.dst_width), static_cast<int>(preview.crop_info.dst_height));

        // Compare the incoming stream against what was requested (resolution logged once from the SPS;
        // fps and bitrate reported together with the latency statistics).
        if (declareParam<bool>("check_stream_format", true)) {
            StreamFormatMonitor::Requested req;
            req.name = resolution_name;
            int fps_int = 0;
            if (std::sscanf(resolution_name.c_str(), "%dx%dp%d", &req.width, &req.height, &fps_int) == 3) {
                req.fps = fps_int;
            }
            req.bitrate = param.video_bitrate;
            format_ = std::make_shared<StreamFormatMonitor>(
                video_format == "h265" ? h26x::Codec::H265 : h26x::Codec::H264, req, node_->get_logger(),
                declareParam<double>("stream_check_fps_tolerance", 0.10),
                declareParam<double>("stream_check_bitrate_tolerance", 0.50));
        }

        delegate_ = std::make_shared<TestStreamDelegate>(node_, video_format, latency_, format_);
        std::shared_ptr<ins_camera::StreamDelegate> delegate = delegate_;
        cam->SetStreamDelegate(delegate);

        auto start = time(NULL);

        uint64_t utc_time = static_cast<uint64_t>(start);
        uint32_t offset_time = 0; //no offset from UTC

        cam->SyncLocalTimeToCamera(utc_time,offset_time);

        cam->StopLiveStreaming();
        RCLCPP_INFO(node_->get_logger(), "Stopped residual live streaming. Waiting 5 seconds for stitcher to subscribe and initialize itself with config and for USB driver to settle");
        rclcpp::sleep_for(std::chrono::seconds(5));

        // Mode, exposure and image settings are applied while streaming is stopped so they are in
        // effect from the first frame.
        if (!applyVideoSubMode() || !applyImageSettings()) {
            return -1;
        }
        applyExposureSettings();

        if (!cam->StartLiveStreaming(param)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start live streaming.");
            return -1;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Live streaming started.");

        if (measure_latency_) {
            // Media time may only advance once streaming is running, so sync after start.
            syncClock();
            using namespace std::chrono;
            clock_sync_timer_ = node_->create_wall_timer(
                duration_cast<nanoseconds>(duration<double>(sync_period_s)), [this]() { syncClock(); });
            latency_report_timer_ = node_->create_wall_timer(
                duration_cast<nanoseconds>(duration<double>(report_period_s)),
                [this]() {
                    latency_->report(node_->get_logger());
                    if (format_) format_->report();
                    const double exp_s = delegate_->lastExposureTime();
                    if (std::isfinite(exp_s) && exp_s > 0.0) {
                        RCLCPP_INFO(node_->get_logger(), "[latency] current exposure_time=%.6f s (1/%.0f s)", exp_s, 1.0 / exp_s);
                    }
                });
            RCLCPP_INFO(node_->get_logger(),
                "[latency] measurement enabled: report every %.1f s, clock sync every %.1f s (%d samples)",
                report_period_s, sync_period_s, clock_sync_samples_);
        }
        return 0;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("insta360_ros_driver");
    
    CameraWrapper camera(node);
    if (camera.run_camera() != 0) {
        rclcpp::shutdown();
        return -1;
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
