#ifndef STORED_IMAGE_PUBLISHER_H
#define STORED_IMAGE_PUBLISHER_H

#include <chrono>
#include <deque>

#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

struct CameraCsv {
    struct CameraData {
        double t;
        std::string filename;
    };

    std::deque<CameraData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            char header_line[2048];
            fscanf(csv, "%2047[^\r]\r\n", header_line);
            char filename_buffer[2048];
            CameraData item;
            while (!feof(csv)) {
                memset(filename_buffer, 0, 2048);
                if (fscanf(csv, "%lf,%2047[^\r]\r\n", &item.t, filename_buffer) != 2) {
                    break;
                }
                item.t *= 1e-9;
                item.filename = std::string(filename_buffer);
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }

        std::sort(items.begin(), items.end(), [](auto &a, auto &b) {
            return a.t < b.t;
        });
    }

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[ns],filename[string]\n", csv);
            for (auto item : items) {
                fprintf(csv, "%ld,%s\n", int64_t(item.t * 1e9), item.filename.c_str());
            }
            fclose(csv);
        }
    }
};

class StoredImagePublisher : public rclcpp::Node {
public:

    StoredImagePublisher(const std::string euroc_path_) : Node("stored_image_publisher"), euroc_path(euroc_path_) {
        camera_csv.load(euroc_path + "/cam0/data.csv");
        RCLCPP_INFO(this->get_logger(), "Image data loaded [%ld]", camera_csv.items.size());

        stored_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("stored_image", 10);
        timer_020Hz = this->create_wall_timer(50ms, std::bind(&StoredImagePublisher::stored_image_publisher_timer_, this));
    }
    
private:

    void stored_image_publisher_timer_() {
        if (camera_csv.items.empty()) {
            RCLCPP_INFO(this->get_logger(), "StoredImages is empty");
            timer_020Hz->cancel();
            return;
        }

        auto item = camera_csv.items.front();
        const std::string filename = euroc_path + "/cam0/data/" + item.filename;
        cv::Mat image = cv::imread(filename);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read image file %s", filename.c_str());
            return;
        }

        auto stored_image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        stored_image_msg_->header.stamp = rclcpp::Time(item.t);

        stored_image_publisher_->publish(*stored_image_msg_);
        RCLCPP_INFO(this->get_logger(), "StoredImage published %s t=%f", filename.c_str(), item.t);
        camera_csv.items.pop_front();
    }

    rclcpp::TimerBase::SharedPtr timer_020Hz;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stored_image_publisher_;

    std::string euroc_path;
    CameraCsv camera_csv;
};

#endif // #ifdef STORED_IMAGE_PUBLISHER_H