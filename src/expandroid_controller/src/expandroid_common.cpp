#include "expandroid_controller/expandroid_common.hpp"

template <>
double get_angle<MotorType::HAND>(
    const expandroid_msgs::msg::ExpandroidState state) {
  return state.hand_angle;
}

template <>
double get_angle<MotorType::X>(
    const expandroid_msgs::msg::ExpandroidState state) {
  return state.x_angle;
}

template <>
double get_angle<MotorType::Y>(
    const expandroid_msgs::msg::ExpandroidState state) {
  return state.y_angle;
}

template <>
double get_angle<MotorType::Z>(
    const expandroid_msgs::msg::ExpandroidState state) {
  return state.z_angle;
}

template <>
double get_speed<MotorType::HAND>(
    const expandroid_msgs::msg::ExpandroidState state) {
  return state.hand_speed;
}

template <>
double get_speed<MotorType::X>(
    const expandroid_msgs::msg::ExpandroidState state) {
  return state.x_speed;
}

template <>
double get_speed<MotorType::Y>(
    const expandroid_msgs::msg::ExpandroidState state) {
  return state.y_speed;
}

template <>
double get_speed<MotorType::Z>(
    const expandroid_msgs::msg::ExpandroidState state) {
  return state.z_speed;
}

template <>
double get_command<MotorType::HAND>(
    const expandroid_msgs::msg::ExpandroidCommand command) {
  return command.hand_command;
}

template <>
double get_command<MotorType::X>(
    const expandroid_msgs::msg::ExpandroidCommand command) {
  return command.x_command;
}

template <>
double get_command<MotorType::Y>(
    const expandroid_msgs::msg::ExpandroidCommand command) {
  return command.y_command;
}

template <>
double get_command<MotorType::Z>(
    const expandroid_msgs::msg::ExpandroidCommand command) {
  return command.z_command;
}
