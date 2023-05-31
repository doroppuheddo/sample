#ifndef RANDOM_IMAGE_PUBLISHER_H
#define RANDOM_IMAGE_PUBLISHER_H

#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class RandomImagePublisher : public rclcpp::Node {
public:
    RandomImagePublisher() : Node("random_image_publisher") {
        random_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
        timer_10Hz = this->create_wall_timer(100ms, std::bind(&RandomImagePublisher::random_image_publisher_timer_, this));
    }
    
private:
    void random_image_publisher_timer_() {
        // Create a new 640x480 image
        cv::Mat image(cv::Size(640, 480), CV_8UC3);
    
        // Generate an image where each pixel is a random color
        cv::randu(image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    
        // Write message to be sent. Member function toImageMsg() converts a CvImage
        // into a ROS image message
        rand_image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    
        // Publish the image to the topic defined in the publisher
        random_image_publisher_->publish(*rand_image_msg_.get());
        RCLCPP_INFO(this->get_logger(), "RandomImage published");
    }

    rclcpp::TimerBase::SharedPtr timer_10Hz;
    sensor_msgs::msg::Image::SharedPtr rand_image_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr random_image_publisher_;
};

#endif // #ifndef RANDOM_IMAGE_PUBLISHER_H