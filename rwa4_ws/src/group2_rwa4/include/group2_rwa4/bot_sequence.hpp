/**
 * @file bot_sequence.hpp
 * @author Pranav Deshakulkarni Manjunath (pdeshaku@umd.edu)
 * @author Pranav ANV (anvpran@umd.edu)
 * @author Lakshmi Pravallika Adibhatla (ladibhat@umd.edu)
 * @brief This file contains the node class definition of type bot_sequence
 * @version 0.1
 * @date 2024-11-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief This class defines a node of name BotSequence
 *
 */
class BotSequence : public rclcpp::Node {
 public:
  /**
   * @brief Constructs a new BotSequence object and initializes its attributes
   *
   */
  BotSequence();

 private:
  /**
   * @brief Defines a (user-defined) enum type called ActionFlag
   *
   */
  enum ActionFlag { MOVE_FORWARD, ROTATE, STOP };

  /**
   * @brief Callback function for /odom topic. The node subscribes to this topic
   * and receives data on this callback function
   *
   * @param msg Contains the message. Is used to read the position and
   * orientation information of the Robot from the Pose field
   */
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief This function is the callback from the wall-timer. It is called once
   * every 500 ms. It is used to send control commands to control the
   * motion of the Robot
   *
   */
  void control_callback();

  /**
   * @brief This function takes in a request to the publish cmd_vel commands and
   * passes it to the publisher
   *
   * @param lin_x Linear velocity in x
   * @param lin_y Linear velocity in y
   * @param lin_z Linear velocity in z
   * @param ang_x Angular velocity in x
   * @param ang_y Angular velocity in y
   * @param ang_z Angular velocity in z
   */
  void move(double lin_x,
            double lin_y,
            double lin_z,
            double ang_x,
            double ang_y,
            double ang_z);
  /**
   * @brief This function calls calls the "move" function with all the speeds
   * set to zero
   *
   */
  void stop();

  /**
   * @brief This is the publisher object for the /cmd_vel topic
   *
   */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  /**
   * @brief This is the subscriber object for the /odom topic
   *
   */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Information regarding the action in progress
   *
   */
  ActionFlag action_flag_;

  /**
   * @brief The current step in the sequence
   *
   */
  int current_step_;

  /**
   * @brief Target distance to travel
   *
   */
  double target_distance_;

  /**
   * @brief Current distance traveled in comparison with the target
   *
   */
  double traveled_distance_;

  /**
   * @brief Target angle to rotate
   *
   */
  double target_angle_;

  /**
   * @brief Curret distance traveled in comparison with the target
   *
   */
  double rotated_angle_;

  /**
   * @brief This is the starting x coordinate before the control moved to this
   * step in the sequence
   *
   */
  double initial_x_;

  /**
   * @brief This is the starting y coordinate before control moved to this step
   * in the sequence
   *
   */
  double initial_y_;

  /**
   * @brief This is the starting yaw angle that the Robot makes before control
   * moved to this step in the sequence
   *
   */
  double initial_yaw_;

  /**
   * @brief The current yaw angle of the Robot
   *
   */
  double yaw_;

  /**
   * @brief The current x,y position of the Robot
   *
   */
  geometry_msgs::msg::Point latest_position_;
};