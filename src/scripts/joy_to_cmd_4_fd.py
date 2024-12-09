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

def calc_current_velocity(value):
    max_velocity = 4.3 # km/hr 4.3 for left and 4.2 for right
    result = value*1/max_velocity
    return result


def joy_callback(data):
    global toggle_state, last_button_state, current_velocity, manual_control, twist

    # Manual control arrows and axes
    axes_3, axes_4, axes_6, axes_7 = data.axes[3], data.axes[4], data.axes[6], data.axes[7]

    # Handle button press for toggle velocity
    if data.buttons[2] == 1 and last_button_state == 0:  # Button just pressed
        if toggle_state == 0:
            current_velocity = calc_current_velocity(4.3) 
            rospy.loginfo(f"Moving with {current_velocity} Km/hr")
            toggle_state = 1
        elif toggle_state == 1:
            current_velocity = calc_current_velocity(4) 
            rospy.loginfo(f"Moving with {current_velocity} Km/hr")
            toggle_state = 2
        elif toggle_state == 2:
            current_velocity = calc_current_velocity(3) 
            rospy.loginfo(f"Moving with {current_velocity} Km/hr")
            toggle_state = 3
        elif toggle_state == 3:
            current_velocity = calc_current_velocity(2) 
            rospy.loginfo(f"Moving with {current_velocity} Km/hr")
            toggle_state = 4
        elif toggle_state == 4:
            current_velocity = calc_current_velocity(1) 
            rospy.loginfo(f"Moving with {current_velocity} Km/hr")
            toggle_state = 5
        elif toggle_state == 5:
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
        else:
            twist.linear.x = axes_4  # Forward/backward joystick
            rospy.loginfo("Forward/Backward joystick pressed")
        
        if axes_6 != 0:
            twist.angular.z = axes_6  # Left/right arrow
            rospy.loginfo("Left/Right arrow pressed")
        else:
            twist.angular.z = axes_3  # Left/right joystick
            rospy.loginfo("Left/Right joystick pressed")
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
    # rospy.loginfo(f"Publishing Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")


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
