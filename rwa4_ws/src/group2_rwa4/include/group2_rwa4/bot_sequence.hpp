#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class BotSequence : public rclcpp::Node {
 public:
  BotSequence();

 private:
  enum ActionFlag { MOVE_FORWARD, ROTATE, STOP };

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_callback();
  void stop();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ActionFlag action_flag_;
  int current_step_;
  double target_distance_;
  double traveled_distance_;
  double target_angle_;
  double rotated_angle_;
  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  double yaw_;
  geometry_msgs::msg::Point latest_position_;
};