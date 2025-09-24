#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
# Modified by Dr. Haghshenas-Jaryani, mahdihj@nmsu.edu, to be use in ME486/586 intro-to-robotics at NMSU
"""
Description: Move line(linear motion)
"""

import os
import sys
import time
from rubikcube_detection_ import *
from scipy.spatial.transform import Rotation as Rot
# from spatialmath import SO3, SE3

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

# initialize the arm (enabling the motion, set mode to 0 (position control), set state to 0)
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)  # mode 0 = position control (check other modes in the documentation)
arm.set_state(state=0)

# send the arm to the home position
arm.move_gohome(wait=True)

# send the arm to the initial position
arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
# get pose information
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_pause_time(5)

# open the gripper
# arm.open_lite6_gripper()
# arm.set_pause_time(5)

# show me your tag
arm.set_position(x=350, y=100, z=150, roll=-180, pitch=-90, yaw=0, speed=50, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
_, tcp_pose_b_t = arm.get_position()
tcp_p_b_t = np.array([tcp_pose_b_t[0:3]])  # the bracket is the trick
tcp_euler_b_t = np.array(tcp_pose_b_t[3:6])
print(tcp_euler_b_t)
print(tcp_p_b_t)
arm.set_pause_time(10)

# april tag detection on the robot arm's wrist
apriltag_point_xyz_m, tags = rubikcube_detection_func()

# print("apriltag_pose_t = ", tags.pose_t)
# print("apriltag_pose_R = ", tags.pose_R)
pose_rot = []

for tag in tags:
    # print("apriltag_pose_R = ", tag.pose_R)
    pose_rot = np.array(tag.pose_R)
    pose_trans = np.dot(np.array(tag.pose_t), 1000)
    # print("pose_rot = ", pose_rot)
    # print("pose_trans = ", pose_trans)
    Rp = np.concatenate((pose_rot, pose_trans), axis=1)
    # print("Rp = ", Rp)
    T_c_a = np.concatenate((Rp, np.array([[0, 0, 0, 1]])), axis=0)
    print("T_c_a = ", T_c_a)

apriltag_point_xyz_mm = np.dot(apriltag_point_xyz_m, 1000)
# print("apriltag_pose_xyz = ", apriltag_point_xyz_mm)

# calculate the camera pose wrt the robot's base
r = Rot.from_euler('xyz', tcp_euler_b_t, degrees=True)
R_b_t = r.as_matrix()
# print("R_b_t = ", R_b_t)
Rp = np.concatenate((R_b_t, tcp_p_b_t.T), axis=1)
# print("Rp = ", Rp)
T_b_t = np.concatenate((Rp, np.array([[0, 0, 0, 1]])), axis=0)
print("T_b_t = ", T_b_t)

# The coordinate system has the origin at the camera center.
# The z-axis points from the camera center out the camera lens.
# The x-axis is to the right in the image taken by the camera, and y is down.
# The tag's coordinate frame is centered at the center of the tag.
# From the viewer's perspective, the x-axis is to the right, y-axis down, and z-axis is into the tag.
x_off = 26.1  # roughly measured by a ruler
y_off = 0
z_off = -56.0
T_t_a = [[0, 0, -1, x_off],[1, 0, 0, y_off],[0, -1, 0, z_off],[0, 0, 0, 1]]
T_b_c = np.dot(T_b_t, np.dot(T_t_a, np.linalg.inv(T_c_a)))
print("T_b_c = ", T_b_c)

# send the arm to the initial position
arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
# get pose information
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_pause_time(10)


# april tag detection on the Rubik's cube(s)
_, tags_cube = rubikcube_detection_func()

for tag in tags_cube:
    # print("apriltag_pose_R = ", tag.pose_R)
    pose_rot = np.array(tag.pose_R)
    pose_trans = np.dot(np.array(tag.pose_t), 1000)
    # print("pose_rot = ", pose_rot)
    # print("pose_trans = ", pose_trans)
    Rp = np.concatenate((pose_rot, pose_trans), axis=1)
    # print("Rp = ", Rp)
    T_c_cube = np.concatenate((Rp, np.array([[0, 0, 0, 1]])), axis=0)
    # print("T_c_a = ", T_c_a)
    T_b_cube = np.dot(T_b_c, T_c_cube)
    # send the arm to the Rubik's cube
    print(T_b_cube.item((0,3)), T_b_cube.item((1,3)), T_b_cube.item((2,3)))
    arm.set_position(x=T_b_cube.item((0,3)), y=T_b_cube.item((1,3)), z=T_b_cube.item((2,3)), roll=-180, pitch=0, yaw=0, speed=50, wait=True)
    arm.set_pause_time(5)
    arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
    arm.set_pause_time(5)


# close the gripper
# arm.close_lite6_gripper()
# arm.set_pause_time(5)

# stop the gripper operation (it is not necessary for the vacuum)
# arm.stop_lite6_gripper()
# arm.set_pause_time(2)

# send the arm to the initial operation position
# arm.set_position(x=150, y=0, z=250, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_pause_time(5)

# send the arm back to the home position
arm.move_gohome(wait=True)
print(arm.get_position())

# disconnect the arm
arm.disconnect()
