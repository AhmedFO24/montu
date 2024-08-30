#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

MAX_RPM = 116.0  # Define the maximum RPM for your motors

def map_range(value, from_min, from_max, to_min, to_max):
    """
    Maps a value from one range to another.
    """
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled_value = (value - from_min) / from_range
    return to_min + (scaled_value * to_range)

def cmd_vel_callback(data):
    """
    Callback function to handle incoming velocity commands.
    Maps joystick values to motor RPMs for skid steering.
    """
    linear_velocity = data.linear.x  # Extract the linear velocity from the Twist message
    angular_velocity = data.angular.z  # Extract the angular velocity from the Twist message

    # Map the joystick values to RPM
    forward_rpm = map_range(linear_velocity, 0.0, 1.0, 0.0, MAX_RPM)  # Forward RPM from 0 to MAX_RPM
    angular_rpm = map_range(abs(angular_velocity), 0.0, 1.0, 0.0, MAX_RPM)   # Angular RPM from 0 to MAX_RPM
    
    # For skid steering:
    # - When moving forward, both motors should be at the same RPM.
    # - When turning, the right motor decreases RPM, and the left motor stays at MAX_RPM.

    # Calculate right and left RPMs based on angular velocity
    if angular_velocity > 0:
        # Turning Left
        rpm_left = forward_rpm - angular_rpm if forward_rpm - angular_rpm > -116 else -116 # Decrease RPM of the right motor
        rpm_right = min(forward_rpm + angular_rpm, MAX_RPM)  # Left motor remains at full RPM
        status_msg = "Turning Left"
    elif angular_velocity < 0:
        # Turning Right
        rpm_left = min(forward_rpm + angular_rpm, MAX_RPM) # Right motor remains at full RPM
        rpm_right = forward_rpm - angular_rpm if forward_rpm - angular_rpm > -116 else -116
        status_msg = "Turning Right"
    else:
        # Moving straight
        rpm_right = forward_rpm
        rpm_left = forward_rpm
        status_msg = "Moving Forward"
    
    if rpm_left < 0 and rpm_right < 0:
        status_msg = "Moving Backward"
    
    if rpm_left == 0 and rpm_right == 0:
        status_msg = "Stopped"

    # Publish the mapped RPM values to the respective topics
    rpm_right_pub.publish(rpm_right)  # Publish the right motor's RPM
    rpm_left_pub.publish(rpm_left)  # Publish the left motor's RPM
    status_pub.publish(status_msg)

def cmd_vel_to_rpm():
    """
    Initializes the node, creates a subscriber to the /cmd_vel topic,
    and publishers to the /rpm_right and /rpm_left topics.
    """
    rospy.init_node('cmd_vel_to_rpm')  # Initialize the ROS node with the name 'cmd_vel_to_rpm'
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)  # Subscribe to the /cmd_vel topic

    global rpm_right_pub, rpm_left_pub, status_pub  # Declare the publishers as global to be accessible in the callback
    rpm_right_pub = rospy.Publisher('/rpm_right', Float32, queue_size=10)  # Create a publisher for /rpm_right
    rpm_left_pub = rospy.Publisher('/rpm_left', Float32, queue_size=10)  # Create a publisher for /rpm_left
    status_pub = rospy.Publisher('/status', String, queue_size=10)  # Create a publisher for /status

    rospy.spin()  # Keep the node running until it is stopped

if __name__ == '__main__':
    cmd_vel_to_rpm()  # Call the function to start the node
