#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Initialize global variables
toggle_state = 0
last_button_state = 0
current_velocity = 0.0
manual_control = True
publish_time = 0.1  # Seconds
twist = Twist()  # Twist message to hold the velocity commands
cmd_vel_pub = None  # Global publisher for velocity commands


def joy_callback(data):
    global toggle_state, last_button_state, current_velocity, manual_control, twist

    # Manual control arrows and axes
    axes_3, axes_4, axes_6, axes_7 = data.axes[3], data.axes[4], data.axes[6], data.axes[7]

    # Handle button press for toggle velocity
    if data.buttons[2] == 1 and last_button_state == 0:  # Button just pressed
        if toggle_state == 0:
            current_velocity = 0.9  # First press: set to 0.9 (6 km/hr)
            rospy.loginfo("Moving with 6 Km/hr")
            toggle_state = 1
        elif toggle_state == 1:
            current_velocity = 0.686  # Second press: set to 0.6 (4 km/hr)
            rospy.loginfo("Moving with 4 Km/hr")
            toggle_state = 2
        elif toggle_state == 2:
            current_velocity = 0.458  # Third press: set to 0.3 (2 km/hr)
            rospy.loginfo("Moving with 2 Km/hr")
            toggle_state = 3
        elif toggle_state == 3:
            current_velocity = 0.0  # Fourth press: stop
            rospy.loginfo("Stopped")
            toggle_state = 0

        manual_control = False  # Disable manual control when button is pressed

    last_button_state = data.buttons[2]  # Update the last button state

    if manual_control:
        # Map joystick axes to linear and angular velocities
        if axes_7 != 0:
            twist.linear.x = axes_7  # Forward/backward arrow
            rospy.loginfo("Forward/Backward arrow pressed")
        elif axes_6 != 0:
            twist.angular.z = axes_6  # Left/right arrow
            rospy.loginfo("Left/Right arrow pressed")
        elif axes_4 != 0:
            twist.linear.x = axes_4  # Forward/backward joystick
            rospy.loginfo("Forward/Backward joystick pressed")
        elif axes_3 != 0:
            twist.angular.z = axes_3  # Left/right joystick
            rospy.loginfo("Left/Right joystick pressed")
        else:
            twist.linear.x = 0
            twist.angular.z = 0  # Stop motion when no input is present

    else:
        # Apply velocity from button presses
        twist.linear.x = current_velocity
        twist.angular.z = 0  # No turning in button control mode

    # Re-enable manual control if any joystick axes are moved
    if axes_7 != 0 or axes_6 != 0 or axes_4 != 0 or axes_3 != 0:
        manual_control = True


def publish_command(event):
    """
    Timer callback to continuously publish velocity commands.
    Ensures that the commands are regularly sent.
    """
    cmd_vel_pub.publish(twist)
    rospy.loginfo(f"Publishing Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")


def joy_to_cmd_vel():
    """
    Initializes the node and sets up subscribers, publishers, and a timer for publishing commands.
    """
    rospy.init_node('joy_to_cmd_vel', anonymous=True)

    # Set up global publisher
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set up subscriber for joystick input
    rospy.Subscriber('/joy', Joy, joy_callback)

    # Timer to regularly call the publish_command function
    rospy.Timer(rospy.Duration(publish_time), publish_command)

    rospy.spin()


if __name__ == '__main__':
    try:
        joy_to_cmd_vel()
    except rospy.ROSInterruptException:
        pass
