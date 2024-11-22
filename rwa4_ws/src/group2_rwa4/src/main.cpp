#include "bot_sequence.hpp"

int main(int argc, char* argv[]) {
  // Initialize the program
  rclcpp::init(argc, argv);

  // Create a new "bot_sequence" node object
  auto node = std::make_shared<BotSequence>();

  // Spin the node to handle and receive callbacks
  rclcpp::spin(node);

  return 0;
}