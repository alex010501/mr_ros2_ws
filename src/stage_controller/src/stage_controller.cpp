/*
 * stage_controller.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: andreyminin
 */

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>

double car_length = 1.5;

double steering = 0;
double cmd_steering = 0.0;
double max_steering_rate = 1.0;
double max_steering = 0.5;
double velocity = 0;
double desired_velocity = 0;
double max_acc = 1.0;
double max_velocity = 10;

rclcpp::Time last_timer_time{};

void on_steering(const std_msgs::msg::Float32& msg) {
  cmd_steering = msg.data;
}

void on_command_velocity(const std_msgs::msg::Float32& msg) {
  desired_velocity = msg.data;
}

double clamp(double cmd, double max_value)
{
  return copysign(std::min(std::abs(cmd), max_value), cmd);
}

double clamp(double value, double cmd, double max_value, double max_rate, double dt) {
  cmd = clamp(cmd, max_value);
  auto diff = (cmd - value);
  diff = clamp(diff, max_rate * dt);
  return value + diff;
}

void on_timer(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_ptr, std::shared_ptr<rclcpp::Node> node) {
  auto t = node->get_clock()->now();
  auto dt = (t - last_timer_time).seconds();
  last_timer_time = t;
  geometry_msgs::msg::Twist cmd;
  velocity = clamp(velocity, desired_velocity, max_velocity, max_acc, dt);
  steering = clamp(steering, cmd_steering, max_steering, max_steering_rate, dt);
  cmd.linear.x = velocity;
  cmd.angular.z = velocity * tan(steering) / car_length;
  twist_pub_ptr->publish(cmd);
}

int main(int argc, char* argv[])
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("stage_throttle_controller");

  node->declare_parameter("length", 1.5);
  node->declare_parameter("max_steering", 0.5);
  node->declare_parameter("max_acc", 1.5);
  node->declare_parameter("max_velocity", 15.0);
  node->declare_parameter("max_steering_rate", 1.0);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_ptr = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  std::function<void()> tim_callback = std::bind(on_timer, twist_pub_ptr, node);
  auto timer = node->create_wall_timer(50ms, tim_callback);
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub = node->create_subscription<std_msgs::msg::Float32>("/steering", 1, on_steering);
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub = node->create_subscription<std_msgs::msg::Float32>("/velocity", 1, on_command_velocity);

  last_timer_time = node->get_clock()->now();
  rclcpp::spin(node);
  return 0;
}


