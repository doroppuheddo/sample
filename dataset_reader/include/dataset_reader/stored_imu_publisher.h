#ifndef STORED_IMU_PUBLISHER_H
#define STORED_IMU_PUBLISHER_H

#include <chrono>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

struct ImuCsv {
    struct ImuData {
        double t;
        struct {
            double x;
            double y;
            double z;
        } w;
        struct {
            double x;
            double y;
            double z;
        } a;
    };

    std::deque<ImuData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            char header_line[2048];
            fscanf(csv, "%2047[^\r]\r\n", header_line);
            ImuData item;
            while (!feof(csv) && fscanf(csv, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n", &item.t, &item.w.x, &item.w.y, &item.w.z, &item.a.x, &item.a.y, &item.a.z) == 7) {
                item.t *= 1e-9;
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
            fputs("#t[ns],w.x[rad/s:double],w.y[rad/s:double],w.z[rad/s:double],a.x[m/s^2:double],a.y[m/s^2:double],a.z[m/s^2:double]\n", csv);
            for (auto item : items) {
                fprintf(csv, "%ld,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n", int64_t(item.t * 1e9), item.w.x, item.w.y, item.w.z, item.a.x, item.a.y, item.a.z);
            }
            fclose(csv);
        }
    }
};

class StoredImuPublisher : public rclcpp::Node {
public:

    StoredImuPublisher(const std::string euroc_path_) : Node("stored_imu_publisher"), euroc_path(euroc_path_) {
        imu_csv.load(euroc_path + "/imu0/data.csv");
        RCLCPP_INFO(this->get_logger(), "Imu data loaded [%ld]", imu_csv.items.size());

        stored_imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("stored_imu", 100);
        timer_200Hz = this->create_wall_timer(5ms, std::bind(&StoredImuPublisher::stored_imu_publisher_timer_, this));
    }
    
private:

    void stored_imu_publisher_timer_() {
        if (imu_csv.items.empty()) {
            RCLCPP_INFO(this->get_logger(), "StoredImu is empty");
            timer_200Hz->cancel();
            return;
        }

        auto item = imu_csv.items.front();

        stored_imu_msg_.header.stamp = rclcpp::Time(item.t);
        stored_imu_msg_.header.frame_id = "imu_frame";
        stored_imu_msg_.orientation.x = 0.0;
        stored_imu_msg_.orientation.y = 0.0;
        stored_imu_msg_.orientation.z = 0.0;
        stored_imu_msg_.orientation.w = 1.0;
        stored_imu_msg_.angular_velocity.x = item.w.x;
        stored_imu_msg_.angular_velocity.y = item.w.y;
        stored_imu_msg_.angular_velocity.z = item.w.z;
        stored_imu_msg_.linear_acceleration.x = item.a.x;
        stored_imu_msg_.linear_acceleration.y = item.a.y;
        stored_imu_msg_.linear_acceleration.z = item.a.z;

        stored_imu_publisher_->publish(stored_imu_msg_);
        RCLCPP_INFO(this->get_logger(), "StoredImu published t=%f", item.t);
        imu_csv.items.pop_front();
    }

    rclcpp::TimerBase::SharedPtr timer_200Hz;
    sensor_msgs::msg::Imu stored_imu_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr stored_imu_publisher_;

    std::string euroc_path;
    ImuCsv imu_csv;
};

#endif // #ifdef STORED_IMU_PUBLISHER_H