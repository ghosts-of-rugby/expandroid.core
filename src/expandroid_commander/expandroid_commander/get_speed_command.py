from sensor_msgs.msg import Joy

from expandroid_msgs.msg import ExpandroidCommand, ExpandroidState


def get_speed_command(joy_msg: Joy, expandroid_command: ExpandroidCommand):
    if joy_msg.buttons[4]:  # A button
        expandroid_command.hand_command = 0.3
    elif joy_msg.buttons[5]:  # B button
        expandroid_command.hand_command = -0.3
    else:
        expandroid_command.hand_command = 0.0

    expandroid_command.x_command = joy_msg.axes[0] * 0.3
    expandroid_command.y_command = joy_msg.axes[1] * 0.3
    expandroid_command.z_command = joy_msg.axes[4] * 0.3

    return expandroid_command


def modify_speed_command(
    expandroid_state: ExpandroidState, expandroid_command: ExpandroidCommand
):
    if (expandroid_state.hand_angle > 1.0) and (expandroid_command.hand_command > 0.0):
        expandroid_command.hand_command = 0.0

    if (expandroid_state.hand_angle < 0.0) and (expandroid_command.hand_command < 0.0):
        expandroid_command.hand_command = 0.0

    if (expandroid_state.x_angle > 1.0) and (expandroid_command.x_command > 0.0):
        expandroid_command.x_command = 0.0

    if (expandroid_state.x_angle < -1.0) and (expandroid_command.x_command < 0.0):
        expandroid_command.x_command = 0.0

    if (expandroid_state.y_angle > 1.0) and (expandroid_command.y_command > 0.0):
        expandroid_command.y_command = 0.0

    if (expandroid_state.y_angle < -1.0) and (expandroid_command.y_command < 0.0):
        expandroid_command.y_command = 0.0

    if (expandroid_state.z_angle > -0.02) and (expandroid_command.z_command > 0.0):
        expandroid_command.z_command = 0.0

    if (expandroid_state.z_angle < -0.98) and (expandroid_command.z_command < 0.0):
        expandroid_command.z_command = 0.0

    return expandroid_command
