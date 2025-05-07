// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

float clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain").get<double>();
  speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset").get<double>();

  steering_min_angle_ = declare_parameter("steering_min_angle").get<double>();
  steering_max_angle_ = declare_parameter("steering_max_angle").get<double>();
  double steering_min_servo = declare_parameter("steering_min_servo").get<double>();
  double steering_max_servo = declare_parameter("steering_max_servo").get<double>();
  steering_center_servo_ = declare_parameter("steering_center_servo").get<double>();

  min_angle_gain_ = steering_min_servo - steering_center_servo_;
  max_angle_gain_ = steering_max_servo - steering_center_servo_;

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  Float64 servo_msg;
  
  servo_msg.data = steering_center_servo_;
  if(cmd->drive.steering_angle > 0){
    double coeff = clip(cmd->drive.steering_angle / steering_max_angle_, 0.0, 1.0);
    servo_msg.data += coeff * max_angle_gain_;
  }
  else if (cmd->drive.steering_angle < 0){
    double coeff = clip(cmd->drive.steering_angle / steering_min_angle_, 0.0, 1.0);
    servo_msg.data += coeff * min_angle_gain_;
  }

  // publish
  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

double AckermannToVesc::clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
