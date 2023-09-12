#include "expandroid_controller/expandroid_common.hpp"

std::array<double, 4> get_angles(
    const expandroid_msgs::msg::ExpandroidState state) {
  return {state.hand_angle, state.x_angle, state.y_angle, state.z_angle};
}

std::array<double, 4> get_speeds(
    const expandroid_msgs::msg::ExpandroidState state) {
  return {state.hand_speed, state.x_speed, state.y_speed, state.z_speed};
}

std::array<double, 4> get_commands(
    const expandroid_msgs::msg::ExpandroidCommand command) {
  return {command.hand_command, command.x_command, command.y_command,
          command.z_command};
}

std::vector<std::array<double, 4>> get_commands(
    const std::vector<expandroid_msgs::msg::ExpandroidCommand> commands) {
  std::vector<std::array<double, 4>> commands_arr;

  for (auto& command : commands) {
    commands_arr.emplace_back(std::array{command.hand_command,
                                         command.x_command, command.y_command,
                                         command.z_command});
  }
  return commands_arr;
}