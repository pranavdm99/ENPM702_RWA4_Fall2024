#include "bot_sequence.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BotSequence>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}