#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Visual Servoing Robot Motion
"""

import os
import sys
import time
# import math
# import serial
# import numpy as np
from rubikcube_detection_ import *



sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('robot.conf')
        ip = parser.get('Lite6', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################
# arduino = serial.Serial(port='/dev/ttyACM0',  baudrate=115200, timeout=0)
# time.sleep(5)

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)


def robo_motion(x_base, y_base, z_base):
    x_inp = x_base  # input("Enter x coordinate = ")
    y_inp = y_base  # input("Enter y coordinate = ")
    z_inp = z_base  # input("Enter z coordinate = ")
    arm.set_position(x=x_inp, y=y_inp, z=z_inp, roll=180, pitch=0, yaw=0, speed=50, wait=True)
    print(arm.get_position(), arm.get_position(is_radian=True))


speed = 20
# arm.set_servo_angle(angle=[45, -20, 0, 0, 20, 0], speed=speed, wait=True)
arm.move_gohome(wait=True)
time.sleep(2)

# get the pose of apriltag attached to the robot arm's wrist
arm.set_servo_angle(angle=[0.0, -19.5, -40.2, 0.0, 59.8, 0.0], speed=speed, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))

# get the pose of the arm (TCP wrt base) and assign it to the variables
tuple_ex = arm.get_position(is_radian=True)
lst_ex = tuple_ex[1]
X = lst_ex[0]
Y = lst_ex[1]
Z = lst_ex[2]
roll = lst_ex[3]
pitch = lst_ex[4]
yaw = lst_ex[5]

# roll rotation (about x)
R3 = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
# pitch rotation (about y)
R2 = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
# yaw rotation (about z)
R1 = np.array([[np.cos(yaw), -np.sin(yaw),0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
# rotation matrix (fixed x-y-z)
R_b_t = np.dot(np.dot(R1, R2), R3)

print("Rotation matrix ", R_b_t)
print("Location in mm: ", X, Y, Z)

# Run Visual Test
apriltag_pose_t_m = []
apriltag_pose_t_mm = []
apriltag_pose_R = []
apriltag_pose_xyz_m, tags = rubikcube_detection_func()

for tag in tags:
    apriltag_pose_t_m.append(tag.pose_t)
    apriltag_pose_R.append(tag.pose_R)


print("Detection Locations")
print("cubes' (x,y,z) @camera in m: ", apriltag_pose_t_m)
# print("edges (x,y,z) @camera in inch: ", edges_m)

# corners_mm = np.dot(corners_m, 1000)
# edges_mm = np.dot(edges_m, 1000)
apriltag_pose_t_mm = np.multiply(apriltag_pose_t_m, 1000)
apriltag_pose_xyz_mm = np.dot(apriltag_pose_xyz_m, 1000)

# define the path
#path_corners = np.vstack((corners_mm, corners_mm[0]))
path_total = apriltag_pose_t_mm

#print("pose_t array size: ", len(apriltag_pose_t_mm))
#print("pose_R array size: ", len(apriltag_pose_xyz_m))
print(apriltag_pose_t_mm)

R_t_c = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
P_r_c = [69.4, -28.0, 25.2]  # based on the right side of the camera
P_offset = np.array([6.0, -16.5, 110])
P_r_c = np.array(P_r_c)
P_toc = np.array([X, Y, Z])

for i in range(len(path_total)):
    print("Tag's location @camera in mm (x,y,z):", path_total[i])
    tags_loc_at_wrist = np.matmul(R_t_c, path_total[i]) + np.expand_dims(P_r_c, axis =1)
    print("Tag's location @wrist in mm (x,y,z):", tags_loc_at_wrist)
    tags_loc_at_base = np.matmul(R_b_t,(np.matmul(R_t_c, path_total[i])) + np.expand_dims(P_r_c, axis =1)) + (np.expand_dims(P_toc, axis=1) - np.matmul(R_t_c,np.expand_dims(P_offset, axis=1)))
    print("Tag's location @base in mm (x,y,z):", tags_loc_at_base)
    print("-------------------------------")

time.sleep(5)
for i in range(len(path_total)):
    tags_loc_at_base = np.matmul(R_b_t,(np.matmul(R_t_c, path_total[i])) + np.expand_dims(P_r_c, axis =1)) + (np.expand_dims(P_toc, axis=1) - np.matmul(R_t_c,np.expand_dims(P_offset, axis=1)))
    robo_motion(tags_loc_at_base[0], tags_loc_at_base[1]-5, tags_loc_at_base[2]+10)
    time.sleep(5)
    # arduino.write(str.encode('1')) #ccw (closing)
    # time.sleep(4)
    arm.set_position(x=0, y=0, z=+100, roll=0, pitch=0, yaw=0, speed=20, relative=True, wait=True)
    time.sleep(5)
    arm.set_position(x=200, y=200, z=200, roll=180, pitch=0, yaw=0, speed=50, wait=True)
    time.sleep(5)
    # arduino.write(str.encode('0')) #cw (opening)
    # time.sleep(2)


# time.sleep(1)
arm.set_servo_angle(angle=[0.0, -57.0, -17.1, 0.0, 74.1, 0.0], speed=speed, wait=True)
time.sleep(1)
arm.set_servo_angle(angle=[45, -20, 0, 0, 20, 0], speed=speed, wait=True)

# send the arm back to the home position
arm.move_gohome(wait=True)
print(arm.get_position())

arm.disconnect()
