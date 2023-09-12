#pragma once

#include "expandroid_msgs/msg/expandroid_command.hpp"
#include "expandroid_msgs/msg/expandroid_state.hpp"

enum class MotorType { HAND, X, Y, Z };

std::array<double, 4> get_angles(const expandroid_msgs::msg::ExpandroidState);

std::array<double, 4> get_speeds(const expandroid_msgs::msg::ExpandroidState);

std::array<double, 4> get_commands(
    const expandroid_msgs::msg::ExpandroidCommand);

std::vector<std::array<double, 4>> get_commands(
    const std::vector<expandroid_msgs::msg::ExpandroidCommand> commands);