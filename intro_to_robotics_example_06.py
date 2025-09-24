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
from shape_detection_func import *

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


def wiggle():
    speed = 30
    angles_w = [2, 0, -2, 0, 2, 0, -2, 0, 2]
    for angle_w in angles_w:
        arm.set_servo_angle(servo_id=6, angle=angle_w, speed=speed, wait=True)
    arm.set_pause_time(2)


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

wiggle()

# -- show me your tag
arm.set_position(x=400, y=100, z=150, roll=-180, pitch=-90, yaw=0, speed=50, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
_, tcp_pose_b_t = arm.get_position()
tcp_p_b_t = np.array([tcp_pose_b_t[0:3]])  # the bracket is the trick
tcp_euler_b_t = np.array(tcp_pose_b_t[3:6])
print(tcp_euler_b_t)
print(tcp_p_b_t)
arm.set_pause_time(10)

# -- april tag detection on the robot arm's wrist
apriltag_point_xyz_m, tags = rubikcube_detection_func(0.017)
pose_rot = []

for tag in tags:
    pose_rot = np.array(tag.pose_R)
    tag.pose_t[2] = apriltag_point_xyz_m[0][2]
    pose_trans = np.dot(np.array(tag.pose_t), 1000)
    # pose_trans = np.dot(apriltag_point_xyz_m, 1000)
    T_c_a = SE3.Rt(pose_rot, pose_trans)
    print("T_c_a = ")
    print(T_c_a)

# --calculate the pose of tcp wrt the robot's base
R_b_t = SO3.RPY(tcp_euler_b_t, unit='deg', order='zyx')
T_b_t = SE3.Rt(R_b_t, tcp_p_b_t.T)
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
T_t_a = SE3.Rt(R_t_a, off_t_a.T)  # [[0, 0, -1, x_off],[1, 0, 0, y_off],[0, -1, 0, z_off],[0, 0, 0, 1]]
T_b_c = T_b_t * (T_t_a * T_c_a.inv())
print("T_b_c = ")
print(T_b_c)
# arm.set_position(x=T_b_c.x, y=T_b_c.y, z=T_b_c.z, roll=-180, pitch=-90, yaw=0, speed=50, wait=True)
arm.set_pause_time(5)

# -- send the arm to the initial position
arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
# -- get pose information
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_pause_time(10)

# -- shape detection and localization
shape_in_the_box = (("Triangle", np.array([253, -199, 170, -180, 0, 0])),
                    ("Square", np.array([203, -232, 170, -180, 0, 0])),
                    ("Pentagon", np.array([0, 0, 0])),
                    ("Hexagon", np.array([0, 0, 0])),
                    ("Octagon", np.array([0, 0, 0])),
                    ("Oval", np.array([0, 0, 0])),
                    ("Parallelogram", np.array([0, 0, 0])),
                    ("Star", np.array([0, 0, 0])),
                    ("Trapezoid", np.array([0, 0, 0])),
                    ("Rectangle", np.array([0, 0, 0])),
                    ("Diamond", np.array([0, 0, 0])),
                    ("Clover", np.array([0, 0, 0])),
                    )
shapes = shape_detection_func()

for shape in shapes:
    print("Shape Name=", shape[0])
    print("Shape centroid position =", shape[1])
    t_c_shape = shape[1]   # shape_pose_in_camera
    R_c_shape = SO3()  # SO3.RPY(0, 0, 0, unit='deg', order='xyz')
    T_c_shape = SE3.Rt(R_c_shape, t_c_shape)  # establish the homogeneous transformation
    T_b_shape = T_b_c * T_c_shape             #
    # -- open the gripper (or turn on the vacuum)
    arm.open_lite6_gripper()
    arm.set_pause_time(2)
    # -- send the arm to the shape
    print(T_b_shape.x, T_b_shape.y, T_b_shape.z)
    arm.set_position(x=T_b_shape.x, y=T_b_shape.y, z=T_b_shape.z + 20, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
    arm.set_pause_time(2)
    arm.set_position(x=T_b_shape.x, y=T_b_shape.y, z=T_b_shape.z, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
    arm.set_pause_time(5)
    # -- close the gripper
    # arm.close_lite6_gripper()
    # arm.set_pause_time(5)
    # -- send the arm back up
    arm.set_position(z=+150, relative=True, wait=True)  # relative to the base frame
    # -- send the arm to the goal pose to place the shap into the box
    target_pose = shape[2]
    arm.set_position(x=target_pose[0], y=target_pose[1], z=target_pose[2] + 50, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
    arm.set_pause_time(2)
    arm.set_position(x=target_pose[0], y=target_pose[1], z=target_pose[2], roll=-180, pitch=0, yaw=0, speed=10, wait=True)
    arm.set_pause_time(2)
    # wiggle()
    # -- close the gripper
    arm.close_lite6_gripper()
    arm.set_pause_time(2)
    arm.set_position(z=+50, relative=True, speed=50, wait=True)  # relative to the base frame
    arm.set_pause_time(2)
    arm.set_position(y=+100, relative=True, speed=50, wait=True)  # relative to the base frame
    arm.set_pause_time(5)

# # -- april tag detection on the Rubik's cube(s)
# tags_point_xyz_m, tags_cube = rubikcube_detection_func(0.017)
# i = 0
# # -- find their pose and send the robot arm to pick them up
# for tag in tags_cube:
#     pose_rot = np.array(tag.pose_R)
#     tag.pose_t[2] = tags_point_xyz_m[i][2]  # replace the depth component of the pose estimation
#     pose_trans = np.dot(np.array(tag.pose_t), 1000)
#     # pose_trans = np.dot(tags_point_xyz_m[i], 1000)
#     T_c_cube = SE3.Rt(pose_rot, pose_trans)  # np.concatenate((Rp, np.array([[0, 0, 0, 1]])), axis=0)
#     T_b_cube = T_b_c * T_c_cube
#     # -- open the gripper (turn on the vacuum)
#     # arm.open_lite6_gripper()
#     # arm.set_pause_time(5)
#     # -- send the arm to the Rubik's cube
#     print(T_b_cube.x, T_b_cube.y, T_b_cube.z)
#     arm.set_position(x=T_b_cube.x, y=T_b_cube.y, z=T_b_cube.z + 20, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
#     arm.set_pause_time(2)
#     arm.set_position(x=T_b_cube.x, y=T_b_cube.y, z=T_b_cube.z, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
#     arm.set_pause_time(5)
#     # -- close the gripper
#     # arm.close_lite6_gripper()
#     # arm.set_pause_time(5)
#     # -- send the arm back up
#     arm.set_position(z=+50, relative=True, wait=True)  # relative to the base frame
#     # -- send the arm to the goal pose
#     arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
#     arm.set_pause_time(5)
#     i = i + 1

# -- stop the gripper operation (it is not necessary for the vacuum)
# arm.stop_lite6_gripper()
# arm.set_pause_time(2)

# -- send the arm to the initial operation position
# arm.set_position(x=150, y=0, z=250, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_pause_time(5)

# -- send the arm back to the home position
arm.move_gohome(wait=True)
print(arm.get_position())

# -- disconnect the arm
arm.disconnect()
