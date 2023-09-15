#pragma once

#include <boost/asio.hpp>
#include <map>
#include <mutex>
#include <thread>

#include "expandroid_msgs/action/trajectory_tracking.hpp"
#include "expandroid_msgs/msg/expandroid_command.hpp"
#include "expandroid_msgs/msg/expandroid_state.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using boost::asio::ip::udp;
using json = nlohmann::json;

struct ExpandroidParameter {
  double c610_current_value_per_ampere = 1000.0;
  double c620_current_value_per_ampere = 16384.0 / 20.0;

  double hand_motor_angle_per_user_angle = 850000.0;
  double hand_motor_speed_per_user_speed =
      4000.0 / 530000 *
      hand_motor_angle_per_user_angle;  // integrated_rpm / measured_angle *
                                        // user_angle

  double x_motor_angle_per_user_angle = 700000.0;
  double x_motor_speed_per_user_speed =
      3900.0 / 536000 * x_motor_angle_per_user_angle;

  double y_motor_angle_per_user_angle = 285000.0;
  double y_motor_speed_per_user_speed =
      1280.0 / 175560 * y_motor_angle_per_user_angle;

  double z_motor_angle_per_user_angle = 1190000.0;
  double z_motor_speed_per_user_speed =
      8468.0 / 1168707 * z_motor_angle_per_user_angle;
};

enum class ControlMode { SPEED_CTRL, TRAJECTORY_TRACKING };

class ExpandroidControlNode : public rclcpp::Node {
  using TrajectoryTracking = expandroid_msgs::action::TrajectoryTracking;
  using GoalHandleTrajectoryTracking =
      rclcpp_action::ServerGoalHandle<TrajectoryTracking>;

 public:
  ExpandroidControlNode();

 private:
  void init();
  void default_update();

  // Trajectory Tracking Action
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const TrajectoryTracking::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleTrajectoryTracking> goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleTrajectoryTracking> goal_handle);

  void execute_trajectory_tracking(
      const std::shared_ptr<GoalHandleTrajectoryTracking> goal_handle);

  expandroid_msgs::msg::ExpandroidState get_expandroid_state();

  void send_speed_command(
      const expandroid_msgs::msg::ExpandroidCommand& command);

  void send_state_command(const expandroid_msgs::msg::ExpandroidState& state);

  rclcpp::TimerBase::SharedPtr default_updater_;

  // State Publisher
  rclcpp::Publisher<expandroid_msgs::msg::ExpandroidState>::SharedPtr
      expandroid_state_publisher_;

  // Speed Command Subscriber
  rclcpp::Subscription<expandroid_msgs::msg::ExpandroidCommand>::SharedPtr
      expandroid_speed_command_subscriber_;

  // Trajectory Tracking Action Server
  rclcpp_action::Server<TrajectoryTracking>::SharedPtr
      trajectory_tracking_action_server_;

  // Socket misc
  boost::asio::io_context io_context_;
  std::unique_ptr<udp::socket> socket_;
  udp::endpoint motor_controller_endpoint_;
  std::mutex socket_mutex_;

  // Expandroid Parameter
  ExpandroidParameter expandroid_parameter_;

  expandroid_msgs::msg::ExpandroidCommand expandroid_speed_command_;
  expandroid_msgs::msg::ExpandroidState expandroid_state_;
  ControlMode control_mode_;
};
