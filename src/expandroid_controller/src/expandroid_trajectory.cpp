#include "expandroid_controller/expandroid_trajectory.hpp"

// Constructor implementation
TotalTrajectory::TotalTrajectory(Trajectory* hand_trajectory,
                                 Trajectory* x_trajectory,
                                 Trajectory* y_trajectory,
                                 Trajectory* z_trajectory)
    : hand_trajectory_(hand_trajectory),
      x_trajectory_(x_trajectory),
      y_trajectory_(y_trajectory),
      z_trajectory_(z_trajectory) {
  // Calculate duration based on the longest duration of the four trajectories
  auto hand_duration = hand_trajectory_->get_duration();
  auto x_duration = x_trajectory_->get_duration();
  auto y_duration = y_trajectory_->get_duration();
  auto z_duration = z_trajectory_->get_duration();
  duration_ = std::max({hand_duration, x_duration, y_duration, z_duration});
}

// Implement the calc_command_state method for TotalTrajectory
expandroid_msgs::msg::ExpandroidState TotalTrajectory::calc_command_state(
    const std::chrono::nanoseconds& time) const {
  expandroid_msgs::msg::ExpandroidState state;
  state.hand_angle = hand_trajectory_->calc_angle(time);
  state.x_angle = x_trajectory_->calc_angle(time);
  state.y_angle = y_trajectory_->calc_angle(time);
  state.z_angle = z_trajectory_->calc_angle(time);
  state.hand_speed = hand_trajectory_->calc_speed(time);
  state.x_speed = x_trajectory_->calc_speed(time);
  state.y_speed = y_trajectory_->calc_speed(time);
  state.z_speed = z_trajectory_->calc_speed(time);
  return state;
}

// Implement the get_duration method for TotalTrajectory
std::chrono::nanoseconds TotalTrajectory::get_duration() const {
  return duration_;
}

// Constructor for ConstantSpeedTrajectory
ConstantSpeedTrajectory::ConstantSpeedTrajectory(const double start_pos,
                                                 const double goal_pos,
                                                 const double abs_speed)
    : start_pos_(start_pos), goal_pos_(goal_pos), abs_speed_(abs_speed) {
  // Calculate duration based on distance and speed
  double distance = std::abs(goal_pos_ - start_pos_);
  std::chrono::duration<double> duration(distance / abs_speed_);
  duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
}

// Implement the calc_angle method for ConstantSpeedTrajectory
double ConstantSpeedTrajectory::calc_angle(
    const std::chrono::nanoseconds& time) const {
  if (time >= duration_) {
    return goal_pos_;
  }
  double distance_covered =
      static_cast<double>(time.count()) / duration_.count();
  return start_pos_ + distance_covered * (goal_pos_ - start_pos_);
}

// Implement the calc_speed method for ConstantSpeedTrajectory
double ConstantSpeedTrajectory::calc_speed(
    const std::chrono::nanoseconds& time) const {
  if (time >= duration_) {
    return 0.0;
  }
  double sign = (goal_pos_ - start_pos_) / std::abs(goal_pos_ - start_pos_);
  return sign * abs_speed_;
}

// Implement the get_duration method for ConstantSpeedTrajectory
std::chrono::nanoseconds ConstantSpeedTrajectory::get_duration() const {
  return duration_;
}

// Constructor for TrapzSpeedTrajectory
TrapzSpeedTrajectory::TrapzSpeedTrajectory(
    const double start_pos, const double goal_pos, const double abs_speed,
    const double constant_speed_time_rate)
    : start_pos_(start_pos),
      goal_pos_(goal_pos),
      abs_speed_(abs_speed),
      constant_speed_time_rate_(constant_speed_time_rate) {
  // Calculate durations
  double distance = std::abs(goal_pos_ - start_pos_);
  double duration =
      (2 * distance) / (abs_speed_ * (1 + constant_speed_time_rate_));
  duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(duration));
}

double TrapzSpeedTrajectory::calc_angle(
    const std::chrono::nanoseconds& time) const {
  if (time >= duration_) {
    return goal_pos_;
  }

  double time_sec = static_cast<double>(time.count()) / 1e9;
  double duration_sec = static_cast<double>(duration_.count()) / 1e9;

  std::chrono::nanoseconds constant_speed_start_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(duration_sec *
                                        (1 - constant_speed_time_rate_) / 2));

  std::chrono::nanoseconds constant_speed_end_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(duration_sec *
                                        (1 + constant_speed_time_rate_) / 2));

  if (time < constant_speed_start_time) {
    return start_pos_ + 0.5 * calc_speed(time) * time_sec;
  } else if (time < constant_speed_end_time) {
    double constant_speed_start_time_sec =
        static_cast<double>(constant_speed_start_time.count()) / 1e9;
    return start_pos_  //
           + 0.5 * calc_speed(constant_speed_start_time) *
                 constant_speed_start_time_sec +
           calc_speed(time) * (time_sec - constant_speed_start_time_sec);
  } else {
    return goal_pos_ - 0.5 * calc_speed(time) * (duration_sec - time_sec);
  }
}

double TrapzSpeedTrajectory::calc_speed(
    const std::chrono::nanoseconds& time) const {
  if (time >= duration_) {
    return 0.0;
  }
  double sign = (goal_pos_ - start_pos_) / std::abs(goal_pos_ - start_pos_);
  double time_sec = static_cast<double>(time.count()) / 1e9;
  double duration_sec = static_cast<double>(duration_.count()) / 1e9;
  if (time_sec < duration_sec * (1 - constant_speed_time_rate_) / 2) {
    return sign * abs_speed_ *
           (time_sec / (duration_sec * (1 - constant_speed_time_rate_) / 2));
  } else if (time_sec < duration_sec * (1 + constant_speed_time_rate_) / 2) {
    return sign * abs_speed_;
  } else {
    return sign * abs_speed_ *
           ((duration_sec - time_sec) /
            (duration_sec * (1 - constant_speed_time_rate_) / 2));
  }
}

std::chrono::nanoseconds TrapzSpeedTrajectory::get_duration() const {
  return duration_;
}

Eigen::Vector4d find_coefficients(double x0, double xT, double T) {
  if (T <= 0) {
    throw std::invalid_argument("T must be positive");
  }

  Eigen::Matrix4d A;
  Eigen::Vector4d b;

  // Fill the matrix A
  A << 1, 0, 0, 0,             // f(0) = x0
      1, T, T * T, T * T * T,  // f(T) = xT
      0, 1, 0, 0,              // f'(0) = 0
      0, 1, 2 * T, 3 * T * T;  // f'(T) = 0

  // Fill the vector b
  b << x0, xT, 0, 0;

  // Solve the system of equations to find the coefficients
  Eigen::Vector4d coefficients = A.colPivHouseholderQr().solve(b);

  return coefficients;
}

CubicTrajectory::CubicTrajectory(const double start_pos, const double goal_pos,
                                 const double speed_max,
                                 const double acceleration_max)
    : start_pos_(start_pos),
      goal_pos_(goal_pos),
      speed_max_(speed_max),
      acceleration_max_(acceleration_max) {
  // Calculate durations
  for (double t = 0.01; t < 10.0; t += 0.01) {
    Eigen::Vector4d coefficients = find_coefficients(start_pos_, goal_pos_, t);
    double speed = coefficients(1) + 2 * coefficients(2) * (t / 2.0) +
                   3 * coefficients(3) * (t / 2.0) * (t / 2.0);
    double acceleration = 2 * coefficients(2);
    if (std::abs(speed) < speed_max_ &&
        std::abs(acceleration) < acceleration_max_) {
      duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(t));
      coefficients_ = coefficients;
      return;
    }
  }
}

double CubicTrajectory::calc_angle(const std::chrono::nanoseconds& time) const {
  if (time >= duration_) {
    return goal_pos_;
  }
  double time_sec = static_cast<double>(time.count()) / 1e9;
  return coefficients_(0) + coefficients_(1) * time_sec +
         coefficients_(2) * time_sec * time_sec +
         coefficients_(3) * time_sec * time_sec * time_sec;
}

double CubicTrajectory::calc_speed(const std::chrono::nanoseconds& time) const {
  if (time >= duration_) {
    return 0.0;
  }
  double time_sec = static_cast<double>(time.count()) / 1e9;
  return coefficients_(1) + 2 * coefficients_(2) * time_sec +
         3 * coefficients_(3) * time_sec * time_sec;
}

std::chrono::nanoseconds CubicTrajectory::get_duration() const {
  return duration_;
}