#include "bot_sequence.hpp"
#include <cmath>

BotSequence::BotSequence()
    : Node("bot_sequence"),
      action_flag_(MOVE_FORWARD),
      current_step_(1),
      target_distance_(10.0),
      traveled_distance_(0.0),
      target_angle_(0.0),
      rotated_angle_(0.0),
      initial_x_(0.0),
      initial_y_(0.0),
      initial_yaw_(0.0),
      yaw_(0.0) {
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&BotSequence::odometry_callback, this, std::placeholders::_1));
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&BotSequence::control_callback, this));
  RCLCPP_INFO(this->get_logger(),
              "BotSequence initialized. Starting sequence.");
}

void BotSequence::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_position_ = msg->pose.pose.position;
  double dx = latest_position_.x - initial_x_;
  double dy = latest_position_.y - initial_y_;
  traveled_distance_ = std::sqrt(dx * dx + dy * dy);

  auto orientation = msg->pose.pose.orientation;
  double siny_cosp =
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y +
                                  orientation.z * orientation.z);
  yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

void BotSequence::control_callback() {
  geometry_msgs::msg::Twist cmd_msg;

  switch (action_flag_) {
    case MOVE_FORWARD:
      if (traveled_distance_ >= target_distance_) {
        stop();
        action_flag_ = STOP;
        if (current_step_ == 1) {
          RCLCPP_INFO(this->get_logger(),
                      "Step 1 finished: Moved 10 m forward");
        } else if (current_step_ == 3) {
          RCLCPP_INFO(this->get_logger(), "Step 3 finished: Moved 5 m forward");
        } else if (current_step_ == 5) {
          RCLCPP_INFO(this->get_logger(),
                      "Step 5 finished: Moved 10 m forward");
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
      } else {
        RCLCPP_INFO(this->get_logger(), "Distance Travelled: %.2f",
                    traveled_distance_);
        cmd_msg.linear.x = 0.5;  // Constant linear velocity
        cmd_pub_->publish(cmd_msg);
      }
      break;

    case ROTATE: {
      double angle_rotated = std::abs(yaw_ - initial_yaw_);
      angle_rotated =
          fmod(angle_rotated + 2 * M_PI, 2 * M_PI);  // Normalize angle
      if (angle_rotated >= target_angle_ - 0.01) {
        stop();
        action_flag_ = STOP;
        if (current_step_ == 2) {
          RCLCPP_INFO(this->get_logger(),
                      "Step 2 finished: Rotated 90 degrees clockwise");
        } else if (current_step_ == 4) {
          RCLCPP_INFO(this->get_logger(),
                      "Step 4 finished: Rotated 45 degrees clockwise");
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
      } else {
        RCLCPP_INFO(this->get_logger(), "Angle rotated: %.2f",
                    angle_rotated);
        cmd_msg.angular.z = -0.5;  // Rotate clockwise
        cmd_pub_->publish(cmd_msg);
      }
      break;
    }

    case STOP:
      switch (current_step_) {
        case 1:  // Completed moving 10 meters
          action_flag_ = ROTATE;
          target_angle_ = 90.0 * M_PI / 180.0;
          initial_yaw_ = yaw_;
          current_step_ = 2;
          break;

        case 2:  // Completed rotating 90 degrees
          action_flag_ = MOVE_FORWARD;
          target_distance_ = 5.0;
          traveled_distance_ = 0.0;
          initial_x_ = latest_position_.x;
          initial_y_ = latest_position_.y;
          current_step_ = 3;
          break;

        case 3:  // Completed moving 5 meters
          action_flag_ = ROTATE;
          target_angle_ = 45.0 * M_PI / 180.0;
          initial_yaw_ = yaw_;
          current_step_ = 4;
          break;

        case 4:  // Completed rotating 45 degrees
          action_flag_ = MOVE_FORWARD;
          target_distance_ = 10.0;
          traveled_distance_ = 0.0;
          initial_x_ = latest_position_.x;
          initial_y_ = latest_position_.y;
          current_step_ = 5;
          break;

        case 5:  // Completed final movement
          RCLCPP_INFO(this->get_logger(),
                      "All tasks completed. Shutting down...");
          rclcpp::shutdown();  // Shut down the program
          break;
      }
      break;
  }
}

void BotSequence::stop() {
  geometry_msgs::msg::Twist cmd_msg;
  cmd_pub_->publish(cmd_msg);  // Send zero velocities to stop
}
