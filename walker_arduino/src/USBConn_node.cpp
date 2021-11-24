#include "walker2_driver/USBConn.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto handle_node = std::make_shared<HandlePublisher>();

  try {
    handle_node->start();
  } catch (const rclcpp::exceptions::RCLError &e) {
    RCLCPP_ERROR( handle_node->get_logger(), "Completely harmless error due to %s", e.what());
  }
 
  RCLCPP_INFO(handle_node->get_logger(), "End publishing measurements at [%s]", handle_node->topic_name_.c_str());
  rclcpp::shutdown();
  return 0;
}