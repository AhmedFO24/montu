#!/usr/bin/env python3
# Author: Tamer Attia (tamer11@vt.edu)
# This code for controlling the Tiger robot
# Calculating the required velocity for each driving motor based on command velocity and kinematics of Tamer UGV
# Please give credit if used

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

# returns the elapsed milliseconds since the start of the program
def millis():
    dt = datetime.now() - start_time
    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
    return ms

# Read what the driver prints 
def get_readings_from_driver():
    status = b''  # Initialize as an empty byte string
    while ser_driver.inWaiting() > 0:
        status = ser_driver.readline()
    return status.decode('ascii') if status else ""  # Decode if status has a value, else return an empty string

# Write the desired speed on the driver using Serial
def motor_write(speed):
    ser_driver.write(speed.encode('ascii'))

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
        status_msg = "Turning Left"
    elif angular_velocity < 0:
        # Turning Right
        rpm_left = min(forward_rpm + angular_rpm, MAX_RPM) # Left motor remains at full RPM
        rpm_right = forward_rpm - angular_rpm if forward_rpm - angular_rpm > -MAX_RPM else -MAX_RPM # Right decreases it's rpm
        status_msg = "Turning Right"
    else:
        # Moving straight
        rpm_right = forward_rpm
        rpm_left = forward_rpm
    
    # Converting Readings from 0 to 1000
    driver_speed[0] = round(((rpm_right + MAX_RPM) * 1000/MAX_RPM) - 1000)
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
    
    #################################### Reading data ###################
    power_data = get_readings_from_driver()
    temp_power_data = power_data.split('\n')   #Split it into an array called dataArray
    power = temp_power_data[0]
    
    ms = millis()
    packet= f"{str(ms)},{power}\n"
    print(packet)
    file.write(packet)


if __name__ == '__main__':
    ser_driver = serial.Serial('/dev/ttyACM1', 115200)
    
    # This command is used to start(!R 1), stop(!R 0) and restart(!R 2) a MicroBasic script if one is loaded in the controller.
    motor_write('!R 1\r')
    
    # sleep for 5 seconds
    rospy.sleep(5)
    
    # Disable Serial Echo (P.341)
    motor_write('^ECHOF 1\r')
    
    # sleep for 1 seconds
    rospy.sleep(1)

    # Open the file in append mode
    file = open("/home/montu/record data/exported_data.txt", "a") # The "a" stands for append. 
    
    # Check if the file is empty
    if os.stat("/home/montu/record data/exported_data.txt").st_size == 0:
        # Write the header line if the file is empty
        file.write("Time,rpm_right,rpm_left,current_right,current_left,power_right,power_left\n")

    start_time = datetime.now()

    print('Start')
    
    rospy.init_node('drive_montu')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

