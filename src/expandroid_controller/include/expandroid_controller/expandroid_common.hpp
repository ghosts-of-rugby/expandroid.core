#pragma once

#include "expandroid_msgs/msg/expandroid_command.hpp"
#include "expandroid_msgs/msg/expandroid_state.hpp"

enum class MotorType { HAND, X, Y, Z };

template <MotorType motor_type>
double get_angle(const expandroid_msgs::msg::ExpandroidState);

template <MotorType motor_type>
double get_speed(const expandroid_msgs::msg::ExpandroidState);

template <MotorType motor_type>
double get_command(const expandroid_msgs::msg::ExpandroidCommand);
