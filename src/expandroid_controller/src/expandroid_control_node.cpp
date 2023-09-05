#include <boost/asio.hpp>
#include <cstdio>
#include <map>

#include "expandroid_msgs/msg/expandroid_command.hpp"
#include "expandroid_msgs/msg/expandroid_state.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
// #include "expandroid_msgss/msg/

using boost::asio::ip::udp;
using json = nlohmann::json;

struct ExpandroidInternalCommand {
  double hand_command;
  double x_command;
  double y_command;
  double z_command;
};

enum class ControlMode { SPEED, ANGLE };

class ExpandroidControlNode : public rclcpp::Node {
 public:
  ExpandroidControlNode() : Node("expandroid_control_node") {
    init();
    control_mode_ = ControlMode::SPEED;
    expandroid_state_publisher_ =
        this->create_publisher<expandroid_msgs::msg::ExpandroidState>(
            "expandroid_state", 1);
    // add ExpandroidCommand subscriber
    expandroid_speed_command_subscriber_ =
        this->create_subscription<expandroid_msgs::msg::ExpandroidCommand>(
            "expandroid_speed_command", 1,
            [this](expandroid_msgs::msg::ExpandroidCommand::UniquePtr msg) {
              control_mode_ = ControlMode::SPEED;
              expandroid_speed_command_.hand_command = msg->hand_command;
              expandroid_speed_command_.x_command = msg->x_command;
              expandroid_speed_command_.y_command = msg->y_command;
              expandroid_speed_command_.z_command = msg->z_command;
            });
    expandroid_angle_command_subscriber_ =
        this->create_subscription<expandroid_msgs::msg::ExpandroidCommand>(
            "expandroid_angle_command", 1,
            [this](expandroid_msgs::msg::ExpandroidCommand::UniquePtr msg) {
              control_mode_ = ControlMode::ANGLE;
              expandroid_angle_command_.hand_command = msg->hand_command;
              expandroid_angle_command_.x_command = msg->x_command;
              expandroid_angle_command_.y_command = msg->y_command;
              expandroid_angle_command_.z_command = msg->z_command;
            });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&ExpandroidControlNode::update, this));

    // rclcpp::create_t
  }

 private:
  void init() {
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

  void update() {
    char buffer[256];
    size_t length = socket_->receive_from(boost::asio::buffer(buffer),
                                          motor_controller_endpoint_);
    std::string message(buffer, length);
    // parse message
    json message_json = json::parse(message);

    auto msg = expandroid_msgs::msg::ExpandroidState();

    const double c610_current_value_per_ampere = 1000.0;
    const double c620_current_value_per_ampere = 16384.0 / 20.0;

    const double hand_motor_angle_per_user_angle = 850000.0;
    const double hand_motor_speed_per_user_speed =
        4000.0 / 530000 * hand_motor_angle_per_user_angle;

    const double x_motor_angle_per_user_angle = 700000.0;
    const double x_motor_speed_per_user_speed =
        3900.0 / 536000 * x_motor_angle_per_user_angle;

    msg.hand_angle = static_cast<double>(message_json[0]["ang"].get<int>()) /
                     hand_motor_angle_per_user_angle;
    msg.hand_speed = static_cast<double>(message_json[0]["spd"].get<int>()) /
                     hand_motor_speed_per_user_speed;
    msg.hand_current =
        message_json[0]["cur"].get<int>() / c610_current_value_per_ampere;

    msg.x_angle = static_cast<double>(message_json[1]["ang"].get<int>()) /
                  x_motor_angle_per_user_angle;
    msg.x_speed = static_cast<double>(message_json[1]["spd"].get<int>()) /
                  x_motor_speed_per_user_speed;
    msg.x_current =
        message_json[1]["cur"].get<int>() / c620_current_value_per_ampere;

    expandroid_state_publisher_->publish(msg);

    // send command
    switch (control_mode_) {
      case ControlMode::SPEED: {
        json command_json = json::array();
        command_json.push_back({
            {"id", 1},
            {"name", "ref_spd"},
            {"value", static_cast<int>(expandroid_speed_command_.hand_command *
                                       hand_motor_speed_per_user_speed)},
        });
        command_json.push_back({
            {"id", 2},
            {"name", "ref_spd"},
            {"value", static_cast<int>(expandroid_speed_command_.x_command *
                                       x_motor_speed_per_user_speed)},
        });
        std::string command_string = command_json.dump();
        socket_->send_to(boost::asio::buffer(command_string),
                         motor_controller_endpoint_);
        break;
      }
      case ControlMode::ANGLE: {
        json command_json = json::array();
        command_json.push_back({
            {"id", 1},
            {"name", "ref_ang"},
            {"value", static_cast<int>(expandroid_angle_command_.hand_command *
                                       hand_motor_angle_per_user_angle)},
        });
        command_json.push_back({
            {"id", 2},
            {"name", "ref_ang"},
            {"value", static_cast<int>(expandroid_angle_command_.x_command *
                                       x_motor_angle_per_user_angle)},
        });
        std::string command_string = command_json.dump();
        socket_->send_to(boost::asio::buffer(command_string),
                         motor_controller_endpoint_);
        break;
      }
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  // add ExpandroidState publisher
  rclcpp::Publisher<expandroid_msgs::msg::ExpandroidState>::SharedPtr
      expandroid_state_publisher_;
  // add ExpandroidCommand subscriber
  rclcpp::Subscription<expandroid_msgs::msg::ExpandroidCommand>::SharedPtr
      expandroid_speed_command_subscriber_;
  rclcpp::Subscription<expandroid_msgs::msg::ExpandroidCommand>::SharedPtr
      expandroid_angle_command_subscriber_;

  boost::asio::io_context io_context_;
  std::unique_ptr<udp::socket> socket_;
  udp::endpoint motor_controller_endpoint_;

  ExpandroidInternalCommand expandroid_speed_command_;
  ExpandroidInternalCommand expandroid_angle_command_;
  ControlMode control_mode_;
};

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExpandroidControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
