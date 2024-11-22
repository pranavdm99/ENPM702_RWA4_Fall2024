#include "bot_sequence.hpp"
#include <cmath>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

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
  // Create a publisher to publish commands to the cmd_vel topic
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Create a subscriber to read the current position from the odom topic and
  // bind it to the callback function odometry_callback
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&BotSequence::odometry_callback, this, std::placeholders::_1));

  // Create a wall timer to control the motion of the Robot based on the
  // required sequence of instructions
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&BotSequence::control_callback, this));

  RCLCPP_INFO(this->get_logger(),
              "BotSequence initialized. Starting sequence.");
}

void BotSequence::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Read the position of the Robot and compute the distance travelled relative
  // to the initial position
  latest_position_ = msg->pose.pose.position;
  double dx = latest_position_.x - initial_x_;
  double dy = latest_position_.y - initial_y_;

  // Calculate the displacement from the initial position. This is given by the
  // Euclidean distance
  traveled_distance_ = std::sqrt(dx * dx + dy * dy);

  // Read the orientation of the Robot and compute the yaw
  tf2::Quaternion quaternion{
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};

  // Convert quaternion to Euler angles
  double roll, pitch, yaw;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  yaw_ = yaw;
}

void BotSequence::control_callback() {
  // This callback is the control loop and it is called once every 500 ms
  // Based on the action flag and the current step, we determine the control
  // action
  switch (action_flag_) {
    case MOVE_FORWARD:
      // The current action flag is MOVE_FORWARD, check if the Robot has
      // traveled the required distance
      if (traveled_distance_ >= target_distance_) {
        // The Robot has traveled the required distance, now publish the
        // command to bring the Robot to a temporary halt before moving to the
        // next step in the sequence
        stop();
        action_flag_ = STOP;  // Set the flag to STOP
        RCLCPP_INFO(this->get_logger(),
                    "Step %d finished: Moved %.2f m forward", current_step_,
                    target_distance_);
      } else {
        // The Robot has not reached the target distance, keep publishing the
        // command to move the Robot forward at constant linear velocity
        move(0.22, 0.0, 0.0, 0.0, 0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Distance Travelled: %.2f",
                    traveled_distance_);
      }
      break;

    case ROTATE: {
      // The current action flag is ROTATE, compute the angle rotated and check
      // if the Robot has rotated by the required angle
      double angle_rotated = std::abs(yaw_ - initial_yaw_);
      angle_rotated =
          fmod(angle_rotated + 2 * M_PI, 2 * M_PI);  // Normalize angle
      if (angle_rotated >= target_angle_) {
        // The Robot has rotated by the required angle, now publish the command
        // to bring the Robot to a temporary halt before moving to the next step
        // in the sequence
        stop();
        action_flag_ = STOP;  // Set the flag to STOP
        RCLCPP_INFO(this->get_logger(),
                    "Step %d finished: Rotated %.2f degrees clockwise",
                    current_step_, target_angle_);

      } else {
        // The Robot has not reached the target orientation, keep publishing the
        // command to rotate the Robot clockwise at constant angular velocity
        move(0.0, 0.0, 0.0, 0.0, 0.0, -0.1);
        RCLCPP_INFO(this->get_logger(), "Angle rotated: %.2f", angle_rotated);
      }
      break;
    }

    case STOP:
      // The current action flag is STOP, so check for the next step in the
      // sequence
      switch (current_step_) {
        case 1:  // Completed moving 10 meters
          // Set the action flag to Rotate and set the target angle to 90
          // degrees. Read the current yaw angle and update the initial yaw,
          // update the current step
          action_flag_ = ROTATE;
          target_angle_ = 90.0 * M_PI / 180.0;
          initial_yaw_ = yaw_;
          current_step_ = 2;
          break;

        case 2:  // Completed rotating 90 degrees
          // Set the action flag to Move Forward and set the target distance to
          // 5 m. Read the current x,y position and update the initial x, y
          // positions, update the current step
          action_flag_ = MOVE_FORWARD;
          target_distance_ = 5.0;
          traveled_distance_ = 0.0;
          initial_x_ = latest_position_.x;
          initial_y_ = latest_position_.y;
          current_step_ = 3;
          break;

        case 3:  // Completed moving 5 meters
          // Set the action flag to Rotate and set the target angle to 45
          // degrees. Read the current yaw angle and update the initial yaw,
          // update the current step
          action_flag_ = ROTATE;
          target_angle_ = 45.0 * M_PI / 180.0;
          initial_yaw_ = yaw_;
          current_step_ = 4;
          break;

        case 4:  // Completed rotating 45 degrees
          // Set the action flag to Move forward and set the target distance to
          // 10 m. Read the current x,y position and update the initial x,y
          // positions, update the current step
          action_flag_ = MOVE_FORWARD;
          target_distance_ = 10.0;
          traveled_distance_ = 0.0;
          initial_x_ = latest_position_.x;
          initial_y_ = latest_position_.y;
          current_step_ = 5;
          break;

        case 5:  // Completed final movement
          // This indicates that the full sequence of movement is complete, shut
          // down the node
          RCLCPP_INFO(this->get_logger(),
                      "All tasks completed. Shutting down...");
          rclcpp::shutdown();  // Shut down the program
          break;
      }
      break;
  }
}

void BotSequence::move(double linx,
                       double liny,
                       double linz,
                       double angx,
                       double angy,
                       double angz) {
  // Take in the request for moving the Robot and publish the command
  geometry_msgs::msg::Twist cmd_msg;
  cmd_msg.angular.x = angx;
  cmd_msg.angular.y = angy;
  cmd_msg.angular.z = angz;
  cmd_msg.linear.x = linx;
  cmd_msg.linear.y = liny;
  cmd_msg.linear.z = linz;

  // Publish the command
  cmd_pub_->publish(cmd_msg);
}

void BotSequence::stop() {
  move(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);  // Send zero velocities to stop
}
