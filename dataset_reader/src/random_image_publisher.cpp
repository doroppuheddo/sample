#include "dataset_reader/random_image_publisher.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
