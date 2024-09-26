#!/usr/bin/env python3
# Author: Tamer Attia (tamer11@vt.edu)
# This code for controlling the Tiger robot
# Calculating the required velocity for each driving motor based on command velocity and kinematics of Tamer UGV
# Please give credit if used

# Use the top button to move with 3 speeds.
# adjust Serial connection /dev/ttyACM0 and /dev/ttyACM1 
# Write data to file and publish it

import rospy
import rospkg
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import operator
import time
import numpy as np
import math
from heapq import *
import serial
import socket
import os 
from datetime import datetime

MAX_RPM = 116  # maximum wheel speed (rpm)
driver_speed = [0]*2 # [0, 0]
# rpm_right_read = 0
# rpm_left_read = 0
# returns the elapsed milliseconds since the start of the program
def millis():
    dt = datetime.now() - start_time
    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
    return ms

# Connect to serial driver
def connect_serial():
    ports = ['/dev/ttyACM0', '/dev/ttyACM1']
    for port in ports:
        try:
            ser_driver = serial.Serial(port, 115200)
            rospy.loginfo(f"Connected to {port}")
            return ser_driver
        except serial.SerialException:
            rospy.loginfo(f"Failed to connect to {port}")
            rospy.sleep(1)
    raise Exception("Could not connect to any available port")

# Read what the driver prints 
def get_readings_from_driver():
    status = b''  # Initialize as an empty byte string
    while ser_driver.inWaiting() > 0:
        status = ser_driver.readline()
    return status.decode('ascii') if status else ""  # Decode if status has a value, else return an empty string

# Write the desired speed on the driver using Serial
def motor_write(speed):
    global ser_driver
    ser_driver.write(speed.encode('ascii'))
    ser_driver.flush()  # Ensure the command is sent immediately

def clean_rpm_data(rpm_values):
    cleaned_values = []
    for value in rpm_values:
        clean_value = value.replace('+', '').replace('\r', '').strip()
        cleaned_values.append(clean_value)
    return cleaned_values

def write_data_to_file_and_publish(event=None):
    global file, rpm_left_read, rpm_right_read
    power_data = get_readings_from_driver()
    
    if power_data:
        temp_power_data = power_data.split('\n')   #Split it into an array called dataArray
        power = temp_power_data[0]
        # ms = millis()
        packet= f"{power}\n"
        rospy.loginfo(packet)
        
        # Extract RPM values
        rpm_values = power.split(',')  # Split the string by commas
        if len(rpm_values) >= 2:  # Ensure there are at least two values
            cleaned_rpm_values = clean_rpm_data(rpm_values)
            try:
                rpm_right_read = int(cleaned_rpm_values[0].strip())  # Convert to int
                rpm_left_read = int(cleaned_rpm_values[1].strip())   # Convert to int
                
                # Publish RPM values
                rpm_right_read_pub.publish(rpm_right_read)
                rpm_left_read_pub.publish(rpm_left_read)
            
            except ValueError:
                rospy.logerr("Error converting RPM values to integers")
                rospy.logerr(f"Received rpm_values: {cleaned_rpm_values}")
                return None
        
        try:
            # Open the file in append mode
            file_path = "/home/montu/record data/exported_data.txt"
            
            # Ensure the directory exists, create it if it doesn't
            directory = os.path.dirname(file_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
            
            with open(file_path, "a") as file:
                # Check if the file is empty
                if os.stat(file_path).st_size == 0:
                    # Write the header line if the file is empty
                    file.write("rpm_right,rpm_left,current_right,current_left,power_right,power_left\n")
                file.write(packet)
        except Exception as e:
            rospy.logerr(f"Error writing to file: {e}")
            return None  # Return None in case of an error
        return packet
    else:
        rospy.logwarn("No power data received")
        return None  # Return None if no data was received

# I think this not used here
def readData():
    buffer_left = ""
    while True:
        oneByte_left = ser_driver.read(1)
        if oneByte_left == b"\r":    #method should returns bytes
            return buffer_left
        else:
            buffer_left += oneByte_left.decode("ascii")

# Map the joystick values to RPM (from (0:1) to (0:max_rpm))
def map_range(value, from_min, from_max, to_min, to_max):
    """
    Maps a value from one range to another.
    """
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled_value = (value - from_min) / from_range
    return to_min + (scaled_value * to_range)

def callback(msg):
    global status_pub
    
    linear_velocity = msg.linear.x  # Extract the linear velocity from the Twist message (Forward and backward of the joystick)
    angular_velocity = msg.angular.z  # Extract the angular velocity from the Twist message (Right and Left of the joystick)

    # Map the joystick values to RPM (from (0:1) to (0:max_rpm))
    forward_rpm = map_range(linear_velocity, 0.0, 1.0, 0.0, MAX_RPM)  # Forward RPM from 0 to MAX_RPM______(Forward and backward of the joystick)
    angular_rpm = map_range(abs(angular_velocity), 0.0, 1.0, 0.0, MAX_RPM)   # Angular RPM from 0 to MAX_RPM______(Right and Left of the joystick)
    
    # For skid steering:
    # - When moving forward, both motors should be at the same RPM.
    # - When turning, the right motor decreases RPM, and the left motor stays at MAX_RPM.

    # Calculate right and left RPMs based on angular velocity
    if angular_velocity > 0:
        # Turning Left
        rpm_left = forward_rpm - angular_rpm if forward_rpm - angular_rpm > -MAX_RPM else -MAX_RPM # Right motor remains at full RPM
        rpm_right = min(forward_rpm + angular_rpm, MAX_RPM)  # Decrease RPM of the left motor
        rospy.loginfo("Turning Left")
        status_msg = "Turning Left"
    elif angular_velocity < 0:
        # Turning Right
        rpm_left = min(forward_rpm + angular_rpm, MAX_RPM) # Left motor remains at full RPM
        rpm_right = forward_rpm - angular_rpm if forward_rpm - angular_rpm > -MAX_RPM else -MAX_RPM # Right decreases it's rpm
        rospy.loginfo("Turning Right")
        status_msg = "Turning Right"
    else:
        # Moving straight
        rpm_right = forward_rpm
        rpm_left = forward_rpm
        rospy.loginfo("Moving Forward")
        status_msg = "Moving Forward"
        
        # difference = rpm_right_read > rpm_left_read
        # if difference:
        #     rpm_left = (MAX_RPM*rpm_left_read) / rpm_right_read
            
    
    if rpm_left < 0 and rpm_right < 0:
        rospy.loginfo("Moving Backward")
        status_msg = "Moving Backward"
    
    if rpm_left == 0 and rpm_right == 0:
        rospy.loginfo("Stopped")
        status_msg = "Stopped"
    
    # Converting Readings from 0 to 1000
    driver_speed[0] = round(((rpm_right + MAX_RPM) * 800/MAX_RPM) - 800)
    driver_speed[1] = round(((rpm_left + MAX_RPM) * 1000/MAX_RPM) - 1000)
    
    # This for preventing Skid Steering
    # if driver_speed[0] < 0 and driver_speed[1] > 0:
    #     driver_speed[0] = 0
    # if driver_speed[1] < 0 and driver_speed[0] > 0:
    #     driver_speed[1] = 0
    
    string_1 = '!VAR 1 ' + str(driver_speed[0]) + '\r'
    string_3 = '!VAR 3 ' + str(driver_speed[1]) + '\r'
    
    flag_drive = 1
    
    if flag_drive == 1:
        vvv = 5
        motor_write(string_1)
        motor_write(string_3)
    else:
        motor_write('!VAR 1 0\r')
        motor_write('!VAR 3 0\r')
    
    status_pub.publish(status_msg)
    # Call write_data_to_file_and_publish continuously while joystick is pressed
    if msg.linear.x or msg.angular.z:
        write_data_to_file_and_publish()  # No argument needed


if __name__ == '__main__':
    rospy.init_node('drive_montu')  # Initialize ROS node at the very beginning
    ser_driver = None
    retry_count = 0
    max_retries = 3  # Limit to 3 retries
    while ser_driver is None and retry_count < max_retries:
        try:
            ser_driver = connect_serial()
        except Exception as e:
            rospy.loginfo(e)
            rospy.sleep(1)  # Retry every second
            retry_count += 1
    
    if ser_driver is None:
        rospy.signal_shutdown("Failed to connect to serial port after maximum retries.")
    else:
        # This command is used to start(!R 1), stop(!R 0) and restart(!R 2) a MicroBasic script if it is loaded in the controller.
        motor_write('!R 1\r')
        
        # sleep for 5 seconds
        rospy.sleep(5)
        
        # Disable Serial Echo (P.341)
        motor_write('^ECHOF 1\r')
        
        # sleep for 1 seconds
        rospy.sleep(1)
        
        start_time = datetime.now()
        rospy.loginfo('Start')
        
        rospy.Subscriber("/cmd_vel", Twist, callback) # Subscribe to /cmd_vel
        status_pub = rospy.Publisher('/status', String, queue_size=10)  # Create a publisher for /status
        rpm_right_read_pub = rospy.Publisher('/rpm_right_read', Int32, queue_size=10)
        rpm_left_read_pub = rospy.Publisher('/rpm_left_read', Int32, queue_size=10)
        
        # Timer to call the function at regular intervals
        # rospy.Timer(rospy.Duration(0.1), write_data_to_file_and_publish)
        
        rospy.spin() # Keep the node running
        file.close()

