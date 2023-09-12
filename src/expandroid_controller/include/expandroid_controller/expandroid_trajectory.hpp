#pragma once

#include <Eigen/Dense>
#include <chrono>

#include "expandroid_controller/expandroid_common.hpp"
#include "expandroid_msgs/msg/expandroid_command.hpp"
#include "expandroid_msgs/msg/expandroid_state.hpp"

class Trajectory {
 public:
  virtual double calc_angle(const std::chrono::nanoseconds& time) const = 0;
  virtual double calc_speed(const std::chrono::nanoseconds& time) const = 0;
  virtual std::chrono::nanoseconds get_duration() const = 0;
};

class TotalTrajectory {
 public:
  TotalTrajectory(Trajectory* hand_trajectory, Trajectory* x_trajectory,
                  Trajectory* y_trajectory, Trajectory* z_trajectory);

  expandroid_msgs::msg::ExpandroidState calc_command_state(
      const std::chrono::nanoseconds& time) const;

  std::chrono::nanoseconds get_duration() const;

 private:
  Trajectory* hand_trajectory_;
  Trajectory* x_trajectory_;
  Trajectory* y_trajectory_;
  Trajectory* z_trajectory_;

  std::chrono::nanoseconds duration_;
};

class ConstantSpeedTrajectory : public Trajectory {
 public:
  /**
   * @brief Construct a new Constant Speed Trajectory object
   *
   * @param start_pos start position
   * @param goal_pos goal position
   * @param abs_speed absolute speed
   */
  ConstantSpeedTrajectory(const double start_pos, const double goal_pos,
                          const double abs_speed);

  double calc_angle(const std::chrono::nanoseconds& time) const override;
  double calc_speed(const std::chrono::nanoseconds& time) const override;
  std::chrono::nanoseconds get_duration() const override;

 private:
  double start_pos_;
  double goal_pos_;
  double abs_speed_;
  std::chrono::nanoseconds duration_;
};

class TrapzSpeedTrajectory : public Trajectory {
 public:
  /*
   * @param start_pos: start position
   * @param goal_pos: goal position
   * @param abs_speed: absolute speed
   * @param constant_speed_time_rate: ratio of constant speed time to total
   * duration
   */
  TrapzSpeedTrajectory(const double start_pos, const double goal_pos,
                       const double abs_speed,
                       const double constant_speed_time_rate);
  double calc_angle(const std::chrono::nanoseconds& time) const override;
  double calc_speed(const std::chrono::nanoseconds& time) const override;
  std::chrono::nanoseconds get_duration() const override;

 private:
  double start_pos_;
  double goal_pos_;
  double abs_speed_;
  double constant_speed_time_rate_;
  std::chrono::nanoseconds duration_;
};

class CubicTrajectory : public Trajectory {
 public:
  CubicTrajectory(const double start_pos, const double goal_pos,
                  const double speed_max, const double acceleration_max);

  double calc_angle(const std::chrono::nanoseconds& time) const override;
  double calc_speed(const std::chrono::nanoseconds& time) const override;
  std::chrono::nanoseconds get_duration() const override;

 private:
  double start_pos_;
  double goal_pos_;
  double speed_max_;
  double acceleration_max_;
  std::chrono::nanoseconds duration_;

  Eigen::Vector4d coefficients_;
};