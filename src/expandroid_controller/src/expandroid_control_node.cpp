#include "expandroid_controller/expandroid_controller.hpp"

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExpandroidControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
