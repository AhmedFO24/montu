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
import threading

sprocket_radius = 0.195/2   # 19.5 is the diameter of the sprocket
vehicle_weight = 112.5 # kg


max_velocity = 4.3 # km/hr 4.3 for left and 4.2 for right
required_velocity = 4.3


MAX_RPM_RIGHT = round(required_velocity*970/max_velocity) # 985 4.2 km/hr maximum wheel speed (rpm)
MAX_RPM_LEFT = round(required_velocity*1000/max_velocity)

driver_speed = [0]*2 # [0, 0]
publish_time = 0.1 #sec
file_name = input("Write the forward speed for exported data: ")

# First thing is to connect to serial
# Connect to serial driver (no change in it)
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

# Giving orders to motors through the driver
def motor_write(order):
    global ser_driver
    ser_driver.reset_input_buffer()  # built in fn from serial module to Clear the input buffer before sending
    ser_driver.write(order.encode('ascii'))
    ser_driver.flush()  # Ensure the command is sent immediately

# Sending the orders in parallel not series
def motor_write_concurrent(order1, order2):
    thread1 = threading.Thread(target=motor_write, args=(order1,))
    thread2 = threading.Thread(target=motor_write, args=(order2,))
    thread2.start()
    thread1.start()
    thread2.join()
    thread1.join()

# Map the joystick values to RPM (from (0:1) to (0:max_rpm))
def map_range(value, from_min, from_max, to_min, to_max):
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled_value = (value - from_min) / from_range
    final_value = to_min + (scaled_value * to_range)
    return final_value

####################################################################################
################################# Readings from driver #############################
# Read what the driver prints 
def get_readings_from_driver():
    status = b''  # Initialize as an empty byte string
    while ser_driver.inWaiting() > 0:
        status = ser_driver.readline()
    return status.decode('ascii') if status else ""  # Decode if status has a value, else return an empty string

# make the readings more clear
def clean_rpm_data(rpm_values):
    cleaned_values = []
    for value in rpm_values:
        clean_value = value.replace('+', '').replace('\r', '').strip()
        cleaned_values.append(clean_value)
    return cleaned_values

#get the time from the driver
# returns the elapsed milliseconds since the start of the program
def millis():
    dt = datetime.now() - start_time
    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
    return ms

def write_data_to_file_and_publish():
    global file
    
    power_data = get_readings_from_driver()
    
    if power_data:
        temp_power_data = power_data.split('\n')   #Split it into an array called dataArray
        power = temp_power_data[0]
        ms = millis()
        
        # Extract RPM values
        rpm_values = power.split(',')  # Split the string by commas
        if len(rpm_values) >= 2:  # Ensure there are at least two values
            cleaned_rpm_values = clean_rpm_data(rpm_values)
            try:
                rpm_right_read = int(cleaned_rpm_values[0].strip()) * (116/1300)  # Convert to int 2320 is 116 rpm and 1300 is motor configuration inside the driver
                rpm_left_read = int(cleaned_rpm_values[1].strip()) * (116/1300)  # Convert to int 2320 is 116 rpm and 1300 is motor configuration inside the driver
                current_right_read = int(cleaned_rpm_values[2].strip())
                current_left_read = int(cleaned_rpm_values[3].strip())
                power_right_read = int(cleaned_rpm_values[4].strip())
                power_left_read = int(cleaned_rpm_values[5].strip())
                velocity_right = int((3.14 * rpm_right_read / 30) * sprocket_radius) # m/sec
                velocity_left = int((3.14 * rpm_left_read / 30) * sprocket_radius) # m/sec
                Right_fd = ((48*30*current_right_read)/(0.1*3.14*rpm_right_read)) / vehicle_weight
                Left_fd =((48*30*current_left_read)/(0.1*3.14*rpm_left_read)) / vehicle_weight
                packet= f"{(int(ms)/1000):.2f},{rpm_right_read},{rpm_left_read},{current_right_read},{current_left_read},{power_right_read},{power_left_read},{Right_fd},{Left_fd},{velocity_right},{velocity_left}\n"
                rospy.loginfo(f"n_R: {rpm_right_read:.2f}RPM, n_L: {rpm_left_read:.2f}RPM, F_R: {Right_fd:.2f}N, F_L: {Left_fd:.2f}N")
                
            
            except (ValueError, ZeroDivisionError):
                # rospy.logerr("Error converting RPM values to integers")
                rospy.logwarn(f"Received rpm_values: {cleaned_rpm_values}")
                return None
        
        try:
            # Open the file in append mode
            file_path = f"/home/montu/record data/straight_{file_name}.txt"
            
            # Ensure the directory exists, create it if it doesn't
            directory = os.path.dirname(file_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
            
            with open(file_path, "a") as file:
                # Check if the file is empty
                if os.stat(file_path).st_size == 0:
                    # Write the header line if the file is empty
                    file.write("time,rpm_right,rpm_left,current_right,current_left,power_right,power_left,fd_right,fd_left,velocity_right,velocity_left\n")
                file.write(packet)
        except Exception as e:
            rospy.loginfo(f"an error occured: {e}")
    else:
        # rospy.logwarn("No power data received")
        return None  # Return None if no data was received

####################################################################################
####################################################################################


# Main Looped Function
def callback(msg):
    # global status_pub
    
    linear_velocity = msg.linear.x  # Extract the linear velocity from the Twist message (Forward and backward of the joystick)
    angular_velocity = msg.angular.z  # Extract the angular velocity from the Twist message (Right and Left of the joystick)

    forward_right = map_range(linear_velocity, 0.0, 1.0, 0.0, MAX_RPM_RIGHT)  # Forward RPM from 0 to MAX_RPM______(Forward and backward of the joystick)
    forward_left = map_range(linear_velocity, 0.0, 1.0, 0.0, MAX_RPM_LEFT)  # Forward RPM from 0 to MAX_RPM______(Forward and backward of the joystick)
    angular_right = map_range(abs(angular_velocity), 0.0, 1.0, 0.0, MAX_RPM_RIGHT)   # Angular RPM from 0 to MAX_RPM______(Right and Left of the joystick)
    angular_left = map_range(abs(angular_velocity), 0.0, 1.0, 0.0, MAX_RPM_LEFT)   # Angular RPM from 0 to MAX_RPM______(Right and Left of the joystick)
    
    # For skid steering:
    # - When moving forward, both motors should be at the same RPM.
    # - When turning, the right motor decreases RPM, and the left motor stays at MAX_RPM.

    if angular_velocity > 0 :
        # Turning Left
        rpm_left = forward_left - angular_left if forward_left - angular_left > -MAX_RPM_LEFT else -MAX_RPM_LEFT # Right motor remains at full RPM
        rpm_right = min(forward_right + angular_right, MAX_RPM_RIGHT)  # Decrease RPM of the left motor
    elif angular_velocity < 0:
        # Turning Right
        rpm_left = min(forward_left + angular_left, MAX_RPM_LEFT) # Left motor remains at full RPM
        rpm_right = forward_right - angular_right if forward_right - angular_right > -MAX_RPM_RIGHT else -MAX_RPM_RIGHT # Right decreases it's rpm
    else:
        # Moving straight
        rpm_right = forward_right
        rpm_left = forward_left
        
    
    
    string_3 = '!VAR 3 ' + str(round(rpm_left)) + '\r'
    string_1 = '!VAR 1 ' + str(round(rpm_right)) + '\r'

    # Function of Sending the orders in parallel not series
    motor_write_concurrent(string_3, string_1)
    
    # Call write_data_to_file_and_publish continuously while joystick is pressed
    if msg.linear.x or msg.angular.z:
        write_data_to_file_and_publish()  # No argument needed
    
# def subscribe_command(event):
#     rospy.Subscriber("/cmd_vel", Twist, callback)
    

if __name__ == '__main__':
    rospy.init_node('drive_montu')  # Initialize ROS node at the very beginning
    
    # connect to driver using Serial
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
        rospy.signal_shutdown("Failed to connect to serial port after 3 retries.")
    else:
        # This command is used to start(!R 1), stop(!R 0) and restart(!R 2) a MicroBasic script if it is loaded in the controller.
        # start giving orders to motors
        motor_write('!R 1\r')
        
        # sleep for 5 seconds
        rospy.sleep(5)
        
        # Disable Serial Echo (P.341)
        motor_write('^ECHOF 1\r')
        
        # sleep for 1 seconds
        rospy.sleep(1)
        
        start_time = datetime.now()
        rospy.loginfo('Start')
        
        while not rospy.is_shutdown():
            rospy.Subscriber("/cmd_vel", Twist, callback) # Subscribe to /cmd_vel
            rospy.sleep(1)
        # rospy.Timer(rospy.Duration(publish_time), subscribe_command) # Subscribe to /cmd_vel)
        
        rospy.spin() # Keep the node running
        file.close()

