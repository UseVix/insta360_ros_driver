#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <mutex>
#include "insta360_ros_driver/msg/camera_compressed_stream_data.hpp"

class GstreamerDecoderNode : public rclcpp::Node {
public:
    GstreamerDecoderNode() : Node("gstreamer_decoder") {
        declare_parameter("use_custom_stream_msg", false);
        bool use_custom_stream_msg = get_parameter("use_custom_stream_msg").as_bool();

        std::string default_topic = use_custom_stream_msg ?
            "/dual_fisheye/image/compressed_stream" :
            "/equirectangular/image/h264";

        declare_parameter("compressed_topic", default_topic);
        declare_parameter("uncompressed_topic", "/equirectangular/image/decoded");
        declare_parameter("pipeline", std::string("appsrc name=source is-live=true format=time caps=video/x-h264,stream-format=byte-stream ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink sync=false"));
        topic_ = get_parameter("compressed_topic").as_string();
        uncompressed_topic_ = get_parameter("uncompressed_topic").as_string();
        pipeline_description_ = get_parameter("pipeline").as_string();
        gst_init(nullptr, nullptr);
        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
        if (error || !pipeline_) throw std::runtime_error(error ? error->message : "GStreamer pipeline failed");
        source_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "source"));
        sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));
        if (!source_ || !sink_) throw std::runtime_error("pipeline needs appsrc name=source and appsink name=sink");
        gst_app_src_set_stream_type(source_, GST_APP_STREAM_TYPE_STREAM);
        g_object_set(G_OBJECT(source_), "format", GST_FORMAT_TIME, nullptr);
        gst_app_sink_set_drop(sink_, true);
        gst_app_sink_set_max_buffers(sink_, 5);
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        publisher_ = create_publisher<sensor_msgs::msg::Image>(uncompressed_topic_, rclcpp::SensorDataQoS());

        // Start dedicated puller thread to decouple GStreamer decoding from ROS callbacks
        running_ = true;
        puller_thread_ = std::thread(&GstreamerDecoderNode::puller_loop, this);

        if (use_custom_stream_msg) {
            custom_subscription_ = create_subscription<insta360_ros_driver::msg::CameraCompressedStreamData>(
                topic_, rclcpp::QoS(10),
                std::bind(&GstreamerDecoderNode::custom_stream_callback, this, std::placeholders::_1));
        } else {
            subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                topic_, rclcpp::QoS(10),
                std::bind(&GstreamerDecoderNode::compressed_image_callback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(get_logger(), "GStreamer Decoder Node initialized");
        RCLCPP_INFO(get_logger(), "Using custom stream message: %s", use_custom_stream_msg ? "true" : "false");
        RCLCPP_INFO(get_logger(), "Subscribing to: %s", topic_.c_str());
        RCLCPP_INFO(get_logger(), "Publishing to: %s", uncompressed_topic_.c_str());
    }
    ~GstreamerDecoderNode() override {
        running_ = false;
        if (puller_thread_.joinable()) {
            puller_thread_.join();
        }
        if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (source_) gst_object_unref(source_);
        if (sink_) gst_object_unref(sink_);
        if (pipeline_) gst_object_unref(pipeline_);
    }
private:
    void puller_loop() {
        while (running_ && rclcpp::ok()) {
            GstSample *sample = gst_app_sink_try_pull_sample(sink_, GST_MSECOND * 50);
            if (!sample) continue;
            GstBuffer *decoded = gst_sample_get_buffer(sample);
            GstMapInfo map;
            if (decoded && gst_buffer_map(decoded, &map, GST_MAP_READ)) {
                GstCaps *caps = gst_sample_get_caps(sample);
                GstStructure *structure = gst_caps_get_structure(caps, 0);
                int width = 0, height = 0;
                gst_structure_get_int(structure, "width", &width);
                gst_structure_get_int(structure, "height", &height);
                if (width > 0 && height > 0 && map.size >= static_cast<size_t>(width * height * 3)) {
                    cv::Mat image(height, width, CV_8UC3, map.data);
                    std_msgs::msg::Header header;
                    {
                        std::lock_guard<std::mutex> lock(header_mutex_);
                        header = latest_header_;
                    }
                    header.stamp = get_clock()->now();
                    publisher_->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
                    decoded_count_++;
                    if (decoded_count_ == 1 || decoded_count_ % 30 == 0) {
                        RCLCPP_INFO(get_logger(), "Decoded %d frames (latest: %dx%d)", decoded_count_, width, height);
                    }
                }
                gst_buffer_unmap(decoded, &map);
            }
            gst_sample_unref(sample);
        }
    }

    void decode_frame(const sensor_msgs::msg::CompressedImage& msg) {
        RCLCPP_INFO_ONCE(get_logger(), "Received first compressed frame! format=%s, size=%zu", msg.format.c_str(), msg.data.size());
        if (msg.format != "h264") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unsupported format: %s", msg.format.c_str());
            return;
        }
        {
            std::lock_guard<std::mutex> lock(header_mutex_);
            latest_header_ = msg.header;
        }
        GstBuffer *buffer = gst_buffer_new_allocate(nullptr, msg.data.size(), nullptr);
        gst_buffer_fill(buffer, 0, msg.data.data(), msg.data.size());
        if (gst_app_src_push_buffer(source_, buffer) != GST_FLOW_OK) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "gst_app_src_push_buffer failed!");
        }
    }

    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        decode_frame(*msg);
    }

    void custom_stream_callback(const insta360_ros_driver::msg::CameraCompressedStreamData::SharedPtr msg) {
        decode_frame(msg->data);
    }

    std::string topic_, uncompressed_topic_, pipeline_description_;
    GstElement *pipeline_ = nullptr;
    GstAppSrc *source_ = nullptr;
    GstAppSink *sink_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Subscription<insta360_ros_driver::msg::CameraCompressedStreamData>::SharedPtr custom_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::thread puller_thread_;
    std::atomic<bool> running_{false};
    std::mutex header_mutex_;
    std_msgs::msg::Header latest_header_;
    int decoded_count_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<GstreamerDecoderNode>()); }
    catch (const std::exception &exc) { fprintf(stderr, "%s\n", exc.what()); }
    rclcpp::shutdown();
    return 0;
}
