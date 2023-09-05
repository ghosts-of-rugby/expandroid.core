#include "expandroid_controller/expandroid_controller.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

ExpandroidControlNode::ExpandroidControlNode()
    : Node("expandroid_control_node") {
  using namespace std::placeholders;

  init();

  control_mode_ = ControlMode::SPEED_CTRL;

  expandroid_state_publisher_ =
      this->create_publisher<expandroid_msgs::msg::ExpandroidState>(
          "expandroid_state", 1);

  expandroid_speed_command_subscriber_ =
      this->create_subscription<expandroid_msgs::msg::ExpandroidCommand>(
          "expandroid_speed_command", 1,
          [this](expandroid_msgs::msg::ExpandroidCommand::UniquePtr msg) {
            control_mode_ = ControlMode::SPEED_CTRL;
            expandroid_speed_command_.hand_command = msg->hand_command;
            expandroid_speed_command_.x_command = msg->x_command;
            expandroid_speed_command_.y_command = msg->y_command;
            expandroid_speed_command_.z_command = msg->z_command;
          });

  trajectory_tracking_action_server_ =
      rclcpp_action::create_server<TrajectoryTracking>(
          this, "trajectory_tracking",
          std::bind(&ExpandroidControlNode::handle_goal, this, _1, _2),
          std::bind(&ExpandroidControlNode::handle_cancel, this, _1),
          std::bind(&ExpandroidControlNode::handle_accepted, this, _1));

  default_updater_ = this->create_wall_timer(
      100ms, std::bind(&ExpandroidControlNode::default_update, this));
}

void ExpandroidControlNode::init() {
  std::lock_guard<std::mutex> lock(socket_mutex_);

  socket_ = std::make_unique<udp::socket>(io_context_,
                                          udp::endpoint(udp::v4(), 1236));

  // initialize endpoint
  motor_controller_endpoint_ = udp::endpoint(
      boost::asio::ip::address::from_string("192.168.137.131"), 1236);
  // send initializtion message
  std::string init_message = "SYN";
  socket_->send_to(boost::asio::buffer(init_message),
                   motor_controller_endpoint_);
  // receive initialization message
  char init_response_buffer[256];
  size_t length = socket_->receive_from(
      boost::asio::buffer(init_response_buffer), motor_controller_endpoint_);
  std::string init_response(init_response_buffer, length);
  // check initialization message
  if (init_response != "ACK") {
    RCLCPP_ERROR(rclcpp::get_logger("ExpandroidControlNode"),
                 "Failed to initialize motor controller.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ExpandroidControlNode"),
                "Successfully initialized motor controller.");
  }
}

void ExpandroidControlNode::default_update() {
  // get state
  expandroid_state_ = get_expandroid_state();

  expandroid_state_publisher_->publish(expandroid_state_);

  // send command
  switch (control_mode_) {
    case ControlMode::SPEED_CTRL: {
      json command_json = json::array();
      command_json.push_back({
          {"id", 1},
          {"name", "ref_spd"},
          {"value", static_cast<int>(
                        expandroid_speed_command_.hand_command *
                        expandroid_parameter_.hand_motor_speed_per_user_speed)},
      });
      command_json.push_back({
          {"id", 2},
          {"name", "ref_spd"},
          {"value", static_cast<int>(
                        expandroid_speed_command_.x_command *
                        expandroid_parameter_.x_motor_speed_per_user_speed)},
      });
      std::string command_string = command_json.dump();
      socket_->send_to(boost::asio::buffer(command_string),
                       motor_controller_endpoint_);
      break;
    }
    case ControlMode::TRAJECTORY_TRACKING: {
      break;
    }
  }
}

rclcpp_action::GoalResponse ExpandroidControlNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const TrajectoryTracking::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExpandroidControlNode::handle_cancel(
    const std::shared_ptr<GoalHandleTrajectoryTracking> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  control_mode_ = ControlMode::SPEED_CTRL;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExpandroidControlNode::handle_accepted(
    const std::shared_ptr<GoalHandleTrajectoryTracking> goal_handle) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&ExpandroidControlNode::execute, this, _1), goal_handle}
      .detach();
}

void ExpandroidControlNode::execute(
    const std::shared_ptr<GoalHandleTrajectoryTracking> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  control_mode_ = ControlMode::TRAJECTORY_TRACKING;
  rclcpp::Rate loop_rate(100ms);
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<TrajectoryTracking::Feedback>();

  for (auto i = 1u; i <= 100; ++i) {
    auto result = std::make_shared<TrajectoryTracking::Result>();
    result->goal_state = expandroid_state_;

    if (goal_handle->is_canceling()) {
      result->goal_state = expandroid_state_;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    // Infomate
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // feedback->progress = i;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    auto result = std::make_shared<TrajectoryTracking::Result>();
    result->goal_state = expandroid_state_;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
  control_mode_ = ControlMode::SPEED_CTRL;
  return;
}

expandroid_msgs::msg::ExpandroidState
ExpandroidControlNode::get_expandroid_state() {
  std::lock_guard<std::mutex> lock(socket_mutex_);

  char buffer[256];
  size_t length = socket_->receive_from(boost::asio::buffer(buffer),
                                        motor_controller_endpoint_);
  std::string message(buffer, length);
  // parse message
  json message_json = json::parse(message);

  auto msg = expandroid_msgs::msg::ExpandroidState();

  msg.hand_angle = static_cast<double>(message_json[0]["ang"].get<int>()) /
                   expandroid_parameter_.hand_motor_angle_per_user_angle;
  msg.hand_speed = static_cast<double>(message_json[0]["spd"].get<int>()) /
                   expandroid_parameter_.hand_motor_speed_per_user_speed;
  msg.hand_current = message_json[0]["cur"].get<int>() /
                     expandroid_parameter_.c610_current_value_per_ampere;

  msg.x_angle = static_cast<double>(message_json[1]["ang"].get<int>()) /
                expandroid_parameter_.x_motor_angle_per_user_angle;
  msg.x_speed = static_cast<double>(message_json[1]["spd"].get<int>()) /
                expandroid_parameter_.x_motor_speed_per_user_speed;
  msg.x_current = message_json[1]["cur"].get<int>() /
                  expandroid_parameter_.c620_current_value_per_ampere;

  return msg;
}

