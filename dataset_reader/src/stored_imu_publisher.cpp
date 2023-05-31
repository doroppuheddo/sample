#include "dataset_reader/stored_imu_publisher.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const std::string euroc_path = "/home/doroppu/ws/data/v1/mav0";

  auto node = std::make_shared<StoredImuPublisher>(euroc_path);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
