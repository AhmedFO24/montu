#!/usr/bin/env python3  

import rospy  # Import rospy for ROS Python bindings
from sensor_msgs.msg import Joy  # Import Joy message type for joystick data
from geometry_msgs.msg import Twist  # Import Twist message type for velocity commands
from std_msgs.msg import Float32, String  # Import message types for RPM and status

MAX_RPM = 116.0  # Define the maximum RPM for the motors

def joy_callback(data):
    """
    Callback function to handle incoming joystick data.
    Converts joystick input into motor RPM and publishes status.
    """
    rpm_right = 0.0  # Initialize right motor RPM
    rpm_left = 0.0   # Initialize left motor RPM
    status_msg = ""  # Initialize status message

    # Check if the axes array has the expected number of elements
    if len(data.axes) < 9:  # Ensure there are enough axes
        rospy.logwarn("Insufficient joystick axes data received.")
        return

    # Check button inputs for movement
    if data.axes[8] == 1:  # Forward button
        rpm_right = MAX_RPM
        rpm_left = MAX_RPM
        status_msg = "Moving Forward"
    elif data.axes[8] == -1:  # Backward button
        rpm_right = -MAX_RPM
        rpm_left = -MAX_RPM
        status_msg = "Moving Backward"
    elif data.axes[7] == -1:  # Right button
        rpm_right = -MAX_RPM
        rpm_left = MAX_RPM
        status_msg = "Turning Right"
    elif data.axes[7] == 1:  # Left button
        rpm_right = MAX_RPM
        rpm_left = -MAX_RPM
        status_msg = "Turning Left"
    else:
        # If no button is pressed, check axes for smooth control
        rpm_right = map_range(data.axes[4], -1.0, 1.0, -MAX_RPM, MAX_RPM)  # Forward/backward axis
        rpm_left = map_range(data.axes[4], -1.0, 1.0, -MAX_RPM, MAX_RPM)   # Forward/backward axis
        rpm_left += map_range(data.axes[3], -1.0, 1.0, -MAX_RPM, MAX_RPM)  # Turning axis
        rpm_right -= map_range(data.axes[3], -1.0, 1.0, -MAX_RPM, MAX_RPM)  # Turning axis
        status_msg = "Controlling with Axes"

    # Clamp RPM values to ensure they stay within the -MAX_RPM to MAX_RPM range
    rpm_right = max(min(rpm_right, MAX_RPM), -MAX_RPM)
    rpm_left = max(min(rpm_left, MAX_RPM), -MAX_RPM)

    # Publish the RPM values and status
    rpm_right_pub.publish(rpm_right)
    rpm_left_pub.publish(rpm_left)
    status_pub.publish(status_msg)

def map_range(value, from_min, from_max, to_min, to_max):
    """
    Maps a value from one range to another.
    """
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled_value = (value - from_min) / from_range
    return to_min + (scaled_value * to_range)

def joy_to_cmd_vel():
    """
    Initializes the node, creates a subscriber to the /joy topic,
    and publishers for RPM and status topics.
    """
    rospy.init_node('joy_to_cmd_vel')  # Initialize the ROS node with the name 'joy_to_cmd_vel'
    rospy.Subscriber('/joy', Joy, joy_callback)  # Subscribe to the /joy topic to receive joystick data

    global rpm_right_pub, rpm_left_pub, status_pub  # Declare the publishers as global to be accessible in the callback
    rpm_right_pub = rospy.Publisher('/rpm_right', Float32, queue_size=10)  # Create a publisher for the /rpm_right topic
    rpm_left_pub = rospy.Publisher('/rpm_left', Float32, queue_size=10)  # Create a publisher for the /rpm_left topic
    status_pub = rospy.Publisher('/status', String, queue_size=10)  # Create a publisher for the /status topic

    rospy.spin()  # Keep the node running until it is stopped

if __name__ == '__main__':
    joy_to_cmd_vel()  # Call the function to start the node
