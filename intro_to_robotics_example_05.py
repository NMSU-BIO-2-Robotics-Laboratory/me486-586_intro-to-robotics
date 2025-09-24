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
from spatialmath import SO3, SE3

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

# --initialize the arm (enabling the motion, set mode to 0 (position control), set state to 0)
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)  # mode 0 = position control (check other modes in the documentation)
arm.set_state(state=0)

# -- send the arm to the home position
arm.move_gohome(wait=True)

# -- send the arm to the initial position
arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
# get pose information
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_pause_time(5)

# -- show me your tag (robot shows the tag to the camera)
arm.set_position(x=350, y=100, z=150, roll=-180, pitch=-90, yaw=0, speed=50, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
_, tcp_pose_b_t = arm.get_position()
tcp_p_b_t = np.array([tcp_pose_b_t[0:3]])  # the bracket is the trick
tcp_euler_b_t = np.array(tcp_pose_b_t[3:6])
print(tcp_euler_b_t)
print(tcp_p_b_t)
arm.set_pause_time(10)

# -- april tag detection on the robot arm's wrist and pose estimation (rotation + position)
apriltag_point_xyz_m, tags = rubikcube_detection_func(0.017)
pose_rot = []

for tag in tags:
    pose_rot = np.array(tag.pose_R)  # orientation of the tag
    tag.pose_t[2] = apriltag_point_xyz_m[0][2] # replacing the depth measurement
    pose_trans = np.dot(np.array(tag.pose_t), 1000)  # position of the tag's center in mm
    T_c_a = SE3.Rt(pose_rot, pose_trans) # assign rotation matrix and position vector into T
    print("T_c_a = ")
    print(T_c_a)

# -- calculate the pose of tcp wrt the robot's base
R_b_t = SO3.RPY(tcp_euler_b_t, unit='deg', order='zyx')  # using roll-pitch-yaw angles to generate the rotation matrix
T_b_t = SE3.Rt(R_b_t, tcp_p_b_t.T)  # assign rotation matrix and position vector into T
print("T_b_t = ")
print(T_b_t)

# --The coordinate system has the origin at the camera center.
# --The z-axis points from the camera center out the camera lens.
# --The x-axis is to the right in the image taken by the camera, and y is down.
# --The tag's coordinate frame is centered at the center of the tag.
# --From the viewer's perspective, the x-axis is to the right, y-axis down, and z-axis is into the tag.
x_off = 26.1  # roughly measured by a ruler
y_off = 0
z_off = -35.5  # for gripper = -48.3 mm and for vacuum = -35.5
off_t_a = np.array([x_off, y_off, z_off])
R_t_a = SO3.RPY(90, -90, 0, unit='deg', order='xyz')
T_t_a = SE3.Rt(R_t_a, off_t_a.T)
T_b_c = T_b_t * (T_t_a * T_c_a.inv())
print("T_b_c (camera with respect to the base) = ")
print(T_b_c)
arm.set_pause_time(5)

# -- send the arm to the initial position
arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
# -- get pose information
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_pause_time(10)


# -- april tag detection on the Rubik's cube(s)
tags_point_xyz_m, tags_cube = rubikcube_detection_func(0.017)  # tag size = 17 mm
i = 0  # tags index set to zero
# -- find their pose
for tag in tags_cube:
    pose_rot = np.array(tag.pose_R)
    tag.pose_t[2] = tags_point_xyz_m[i][2]  # replace the depth component of the pose estimation
    pose_trans = np.dot(np.array(tag.pose_t), 1000)
    T_c_cube = SE3.Rt(pose_rot, pose_trans)
    T_b_cube = T_b_c * T_c_cube
    print(T_b_cube.x, T_b_cube.y, T_b_cube.z)  # x, y,and z elements of the position vector
    arm.set_pause_time(5)
    i = i + 1  # tags index increase by one

# -- send the arm back to the home position
arm.move_gohome(wait=True)
print(arm.get_position())

# -- disconnect the arm
arm.disconnect()
