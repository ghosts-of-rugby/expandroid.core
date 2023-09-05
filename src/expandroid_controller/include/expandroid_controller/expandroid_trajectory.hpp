#pragma once

#include <chrono>

#include "expandroid_controller/expandroid_common.hpp"
#include "expandroid_msgs/msg/expandroid_command.hpp"
#include "expandroid_msgs/msg/expandroid_state.hpp"

class MotorPlanner {
 public:
  MotorPlanner(const expandroid_msgs::msg::ExpandroidState& initial_state,
               const expandroid_msgs::msg::ExpandroidCommand& goal_pos,
               const double& hand_speed, const double& x_speed,
               const double& y_speed, const double& z_speed);
  template <MotorType motor_type>
  double calc_angle(const std::chrono::nanoseconds& time) const {
    double required_time_second = get_required_time_of_motor<motor_type>();
    double current_time_second = time.count() / 1e9;

    if (current_time_second <= 0.0) {
      return get_angle<motor_type>(initial_state_);
    } else if (current_time_second < required_time_second) {
      return get_angle<motor_type>(initial_state_) +
             get_direction<motor_type>() * get_speed_param<motor_type>() *
                 time.count() / 1e9;
    } else {
      return get_command<motor_type>(goal_pos_);
    }
  }

  template <MotorType motor_type>
  double calc_speed(const std::chrono::nanoseconds& time) {
    double required_time_second = get_required_time_of_motor<motor_type>();
    double current_time_second = time.count() / 1e9;

    if (0.0 <= current_time_second &&
        current_time_second < required_time_second) {
      return get_direction<motor_type>() * get_speed_param<motor_type>();
    }

    return 0.0;
  }

  std::chrono::nanoseconds get_duration() const;

 private:
  const expandroid_msgs::msg::ExpandroidState initial_state_;
  const expandroid_msgs::msg::ExpandroidCommand goal_pos_;

  const double hand_speed_;
  const double x_speed_;
  const double y_speed_;
  const double z_speed_;

  std::chrono::nanoseconds duration_;

  template <MotorType motor_type>
  double get_speed_param() const;

  template <MotorType motor_type>
  double get_distance_error() const {
    return get_command<motor_type>(goal_pos_) -
           get_angle<motor_type>(initial_state_);
  }

  template <MotorType motor_type>
  double get_required_time_of_motor() const {
    return std::abs(get_distance_error<motor_type>()) /
           get_speed_param<motor_type>();
  }

  template <MotorType motor_type>
  double get_direction() const {
    double distance_error = get_distance_error<motor_type>();
    if (distance_error < 0.0) {
      return -1.0;
    } else if (distance_error > 0.0) {
      return 1.0;
    } else {
      return 0.0;
    }
  }
};

template <>
double MotorPlanner::get_speed_param<MotorType::X>() const;
template <>
double MotorPlanner::get_speed_param<MotorType::Y>() const;
template <>
double MotorPlanner::get_speed_param<MotorType::Z>() const;
template <>
double MotorPlanner::get_speed_param<MotorType::HAND>() const;

class Trajectory {
 public:
  virtual double calc_angle(const std::chrono::nanoseconds& time) const = 0;
  virtual double calc_speed(const std::chrono::nanoseconds& time) const = 0;
  virtual std::chrono::nanoseconds get_duration() const = 0;
};

class ConstantSpeedTrajectory : public Trajectory {
 public:
  ConstantSpeedTrajectory(const double start_pos, const double goal_pos,
                          const double abs_speed)
      : start_pos_(start_pos), goal_pos_(goal_pos), abs_speed_(abs_speed) {
    // Calculate duration based on distance and speed
    double distance = std::abs(goal_pos_ - start_pos_);
    std::chrono::duration<double> duration(distance / abs_speed_);
    duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
  }

  // Implement the Trajectory interface
  double calc_angle(const std::chrono::nanoseconds& time) const override {
    if (time >= duration_) {
      return goal_pos_;
    }
    double distance_covered =
        static_cast<double>(time.count()) / duration_.count();
    return start_pos_ + distance_covered * (goal_pos_ - start_pos_);
  }

  double calc_speed(const std::chrono::nanoseconds& time) const override {
    if (time >= duration_) {
      return 0.0;
    }
    return abs_speed_;
  }

  std::chrono::nanoseconds get_duration() const override { return duration_; }

 private:
  double start_pos_;
  double goal_pos_;
  double abs_speed_;
  std::chrono::nanoseconds duration_;
};