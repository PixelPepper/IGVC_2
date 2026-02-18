/**
 * Copyright (C) 2023  kbkn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <chrono>

using namespace std::chrono_literals;

class CmdVelOverrideNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  serial::Serial ser_;
  std::string input_topic_;
  std::string output_topic_;
  std::string port_;
  bool estop_;
  int baudrate_;
  int time_out_;
  double serial_interval_;

public:
  CmdVelOverrideNode();
  void openSerial(const std::string& port, const int& baudrate, const int& time_out);
  void timerCallback();
  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg);
};
