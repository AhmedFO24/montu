#!/usr/bin/env python3  

import rospy  # Import rospy for ROS Python bindings
from sensor_msgs.msg import Joy  # Import Joy message type for joystick data
from geometry_msgs.msg import Twist  # Import Twist message type for velocity commands

# Initialize a state variable to track the toggle state
toggle_state = 0
last_button_state = 0  # Tracks the last state of the button (whether it was pressed or not)
current_velocity = 0.0  # Store the current velocity based on button presses
manual_control = True  # Track if joystick manual control is active

def joy_callback(data):
    global toggle_state, last_button_state, current_velocity, manual_control
    """
    Callback function to handle incoming joystick data.
    Converts joystick input into a Twist message.
    """
    twist = Twist()  # Create a new Twist message instance
    
    # Check if button 2 is pressed (and detect if it was released and pressed again)
    if data.buttons[2] == 1 and last_button_state == 0:  # Button just pressed
        # Toggle velocity
        if toggle_state == 0:
            current_velocity = 0.9  # First press: set to 0.9 (6 km/hr)
            rospy.loginfo("Moving with 6 Km/hr")
            toggle_state = 1
        elif toggle_state == 1:
            current_velocity = 0.6  # Second press: set to 0.6 (4 km/hr)
            rospy.loginfo("Moving with 4 Km/hr")
            toggle_state = 2
        elif toggle_state == 2:
            current_velocity = 0.3  # Third press: set to 0.3 (2 km/hr)
            rospy.loginfo("Moving with 2 Km/hr")
            toggle_state = 3
        elif toggle_state == 3:
            current_velocity = 0.0  # Fourth press: stop
            rospy.loginfo("Stopped")
            toggle_state = 0
        
        # Disable manual joystick control when the button is pressed
        manual_control = False
    
    # Update the last button state
    last_button_state = data.buttons[2]
    
    # If manual control is active, use joystick axes for control
    if manual_control:
        # Map joystick axes to linear and angular velocities
        if data.axes[7] != 0:  # Check if the forward/backward button is pressed
            twist.linear.x = data.axes[7]  # Use axes[7] for primary movement
            rospy.loginfo("Axes[7]")
        else:
            twist.linear.x = data.axes[4]  # Otherwise use axes[4] for control
            rospy.loginfo("Axes[4]")
        
        if data.axes[6] != 0:  # Check if the right/left button is pressed
            twist.angular.z = data.axes[6]  # Use axes[6] for primary turning
            rospy.loginfo("Axes[6]")
        else:
            twist.angular.z = data.axes[3]  # Otherwise use axes[3] for control
            rospy.loginfo("Axes[3]")
    else:
        # Apply the current velocity set by the button presses
        twist.linear.x = current_velocity
        twist.angular.z = 0  # Set angular velocity to zero when in button control mode

    # Publish the Twist message to the /cmd_vel topic
    cmd_vel_pub.publish(twist)
    
    # If axes are moved, re-enable manual control
    if data.axes[7] != 0 or data.axes[6] != 0 or data.axes[4] != 0 or data.axes[3] != 0:
        # rospy.loginfo("Manual control re-activated via joystick movement")
        manual_control = True

def joy_to_cmd_vel():
    """
    Initializes the node, creates a subscriber to the /joy topic,
    and a publisher to the /cmd_vel topic.
    """
    rospy.init_node('joy_to_cmd_vel')  # Initialize the ROS node with the name 'joy_to_cmd_vel'
    rospy.Subscriber('/joy', Joy, joy_callback)  # Subscribe to the /joy topic to receive joystick data

    global cmd_vel_pub  # Declare the cmd_vel_pub as global to be accessible in the callback
    #######################################################################################################
    ########################### Adding this part For Continuous Publishing the topic ######################

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Create a publisher for the /cmd_vel topic
    rospy.sleep(1)
    ########################################################################################################
    ########################################################################################################

    rospy.spin()  # Keep the node running until it is stopped

if __name__ == '__main__':
    joy_to_cmd_vel()  # Call the function to start the node
