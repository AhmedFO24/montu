#!/usr/bin/env python3  

import rospy  # Import rospy for ROS Python bindings
from sensor_msgs.msg import Joy  # Import Joy message type for joystick data
from geometry_msgs.msg import Twist  # Import Twist message type for velocity commands

def joy_callback(data):
    """
    Callback function to handle incoming joystick data.
    Converts joystick input into a Twist message.
    """
    twist = Twist()  # Create a new Twist message instance

    # Map joystick axes to linear and angular velocities
    twist.linear.x = data.axes[4]  # Map the forward/backward joystick axis (assumed index 1) to linear velocity
    twist.angular.z = data.axes[3]  # Map the left/right joystick axis (assumed index 0) to angular velocity

    cmd_vel_pub.publish(twist)  # Publish the Twist message to the /cmd_vel topic

def joy_to_cmd_vel():
    """
    Initializes the node, creates a subscriber to the /joy topic,
    and a publisher to the /cmd_vel topic.
    """
    rospy.init_node('joy_to_cmd_vel')  # Initialize the ROS node with the name 'joy_to_cmd_vel'
    rospy.Subscriber('/joy', Joy, joy_callback)  # Subscribe to the /joy topic to receive joystick data

    global cmd_vel_pub  # Declare the cmd_vel_pub as global to be accessible in the callback
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Create a publisher for the /cmd_vel topic

    rospy.spin()  # Keep the node running until it is stopped

if __name__ == '__main__':
    joy_to_cmd_vel()  # Call the function to start the node
