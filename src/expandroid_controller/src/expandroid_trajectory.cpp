#include "expandroid_controller/expandroid_trajectory.hpp"

// Constructor implementation
MotorPlanner::MotorPlanner(
    const expandroid_msgs::msg::ExpandroidState& initial_state,
    const expandroid_msgs::msg::ExpandroidCommand& goal_pos,
    const double& hand_speed, const double& x_speed, const double& y_speed,
    const double& z_speed)
    : initial_state_(initial_state),
      goal_pos_(goal_pos),
      hand_speed_(hand_speed),
      x_speed_(x_speed),
      y_speed_(y_speed),
      z_speed_(z_speed) {
  double hand_required_time_second =
      get_required_time_of_motor<MotorType::HAND>();
  double x_required_time_second = get_required_time_of_motor<MotorType::X>();
  double y_required_time_second = get_required_time_of_motor<MotorType::Y>();
  double z_required_time_second = get_required_time_of_motor<MotorType::Z>();

  double max_required_time_second =
      std::max({hand_required_time_second, x_required_time_second,
                y_required_time_second, z_required_time_second});
  duration_ = std::chrono::nanoseconds(
      static_cast<int64_t>(max_required_time_second * 1e9));
}

std::chrono::nanoseconds MotorPlanner::get_duration() const {
  return duration_;
}

template <>
double MotorPlanner::get_speed_param<MotorType::HAND>() const {
  return hand_speed_;
}

template <>
double MotorPlanner::get_speed_param<MotorType::X>() const {
  return x_speed_;
}

template <>
double MotorPlanner::get_speed_param<MotorType::Y>() const {
  return y_speed_;
}

template <>
double MotorPlanner::get_speed_param<MotorType::Z>() const {
  return z_speed_;
}
