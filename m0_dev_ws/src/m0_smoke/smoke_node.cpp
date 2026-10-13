#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("m0_smoke");
  auto pub = node->create_publisher<std_msgs::msg::String>("m0_smoke", 10);
  auto timer = node->create_wall_timer(std::chrono::milliseconds(200), [&]() {
    std_msgs::msg::String msg; msg.data = "m0_workspace_ok"; pub->publish(msg);
  });
  rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
