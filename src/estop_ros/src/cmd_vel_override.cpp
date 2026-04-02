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

#include "cmd_vel_override/cmd_vel_override.h"

CmdVelOverrideNode::CmdVelOverrideNode() : rclcpp::Node("cmd_vel_override_node")
{
  this->declare_parameter<std::string>("port", "/dev/ttyACM0");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<int>("time_out", 500);
  this->declare_parameter<double>("timer_interval", 0.01);
  this->declare_parameter<std::string>("input_topic", "cmd_vel/raw");
  this->declare_parameter<std::string>("output_topic", "cmd_vel/override");
  this->get_parameter("port", port_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("time_out", time_out_);
  this->get_parameter("serial_interval", serial_interval_);
  this->get_parameter("input_topic", input_topic_);
  this->get_parameter("output_topic", output_topic_);
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, 10, std::bind(&CmdVelOverrideNode::cmdVelCallback, this, std::placeholders::_1));
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
  estop_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("estop/state", 10);
  timer_ = this->create_wall_timer(std::chrono::duration<double>(serial_interval_),
                                   std::bind(&CmdVelOverrideNode::timerCallback, this));
  estop_ = false;
  openSerial(port_, baudrate_, time_out_);
}

void CmdVelOverrideNode::openSerial(const std::string& port, const int& baudrate, const int& time_out)
{
  RCLCPP_INFO(this->get_logger(), "Port:%s baud:%d", port.c_str(), baudrate);
  serial::Timeout to = serial::Timeout::simpleTimeout(time_out);

  while (rclcpp::ok())
  {
    try
    {
      // Attempt to open the serial port
      ser_.setPort(port);
      ser_.setBaudrate(baudrate);
      ser_.setTimeout(to);
      ser_.open();
      RCLCPP_INFO(this->get_logger(), "\033[32mSerial port opened successfully!\033[0m");
      break;
    }
    catch (serial::IOException& e)
    {
      RCLCPP_WARN(this->get_logger(), "Serial port opening failure...Retrying");
      rclcpp::sleep_for(5s);
    }
  }
}

void CmdVelOverrideNode::timerCallback()
{
  try
  {
    if (ser_.available())
    {
      uint8_t byte = ser_.read(1)[0];
      if (byte == 0x01)
      {
        estop_ = true;
      }
      else if (byte == 0x00)
      {
        estop_ = false;
      }
    }

    std_msgs::msg::Bool estop_state_msg;
    estop_state_msg.data = estop_;
    estop_state_pub_->publish(estop_state_msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(this->get_logger(), "E-STOP disconnect...Retrying");
    openSerial(port_, baudrate_, time_out_);
  }
}

void CmdVelOverrideNode::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg)
{
  geometry_msgs::msg::Twist pub_twist_msg = *twist_msg;

  if (estop_)
  {
    pub_twist_msg.linear.x = 0.0;
    pub_twist_msg.angular.z = 0.0;
    twist_pub_->publish(pub_twist_msg);
  }
  else
  {
    twist_pub_->publish(pub_twist_msg);
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelOverrideNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
