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
from image_capture_ import *
from scipy.spatial.transform import Rotation as Rot
from spatialmath import SO3, SE3
import pyrealsense2 as rs

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
arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=150, wait=True)
# get pose information
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_pause_time(0.1)

# -- show me your tag (robot shows the tag to the camera)
arm.set_position(x=350, y=0, z=150, roll=-180, pitch=-90, yaw=0, speed=150, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
_, tcp_pose_b_t = arm.get_position()
tcp_p_b_t = np.array([tcp_pose_b_t[0:3]])  # the bracket is the trick
tcp_euler_b_t = np.array(tcp_pose_b_t[3:6])
print(tcp_euler_b_t)
print(tcp_p_b_t)
arm.set_pause_time(0.1)

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
arm.set_pause_time(0.1)

# -- send the arm to the initial position
arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=150, wait=True)
# -- get pose information
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_pause_time(0.1)


# -- april tag detection on the Rubik's cube(s)
#tags_point_xyz_m, tags_cube = rubikcube_detection_func(0.017)  # tag size = 17 mm
#---------------------------------------------------------------------------------------------------------
# take an RGB image save it as shapes.png
#depth_frame, depth_intrinsics = image_capture()
depth_frame, depth_intrinsics, color_intrinsics = image_capture()

# reading image
img_org = cv2.imread('shapes.png')
cv2.imshow('original',img_org)

print("Shape of the image", img_org.shape)

# [rows, columns]
img = img_org[200:400, 200:500]  # [ymin:ymax, xmin:xmax] [480, 640]
y_min_offset = 200  # x_min
x_min_offset = 200 + 10 # y_min
# img = img_org[125:370, 310:640]  # [ymin:ymax, xmin:xmax] [480, 640]
# y_min_offset = 125   # x_min
# x_min_offset = 310   # y_min

cv2.imshow('cropped', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# converting image into blurred image
blur = cv2.GaussianBlur(img, (7,7), 0)
cv2.imshow('blur',blur)

# converting image into grayscale image
gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
cv2.imshow('gray',gray)
#cv2.waitKey(0)

# setting threshold of gray image
_, threshold = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY)
cv2.imshow('threshold',threshold)

# using a findContours() function
contours, _ = cv2.findContours(
    threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#contoursalt = contours

#print(len(contours))

i = 0
offset = 0
height = 40 # height to go to when picking up block

# for contouralt in contoursalt:
# # here we are ignoring first counter because
#     # findcontour function detects whole image as shape
#     if i == 0:
#         i = 1
#         continue
#
#     # cv2.approxPloyDP() function to approximate the shape
#     approx = cv2.approxPolyDP(
#         contouralt, 0.05 * cv2.arcLength(contouralt, True), True)
#     print(len(approx))
#
#     # using drawContours() function
#     cv2.drawContours(img, [contouralt], 0, (0, 0, 255), 5)
#
#     # finding the center point of shape
#     M = cv2.moments(contouralt)
#     #print(M['m00'])
#     if M['m00'] != 0.0:
#         x = int(M['m10'] / M['m00'])
#         y = int(M['m01'] / M['m00'])
#     print("x, y = ", x + x_min_offset, y + y_min_offset)
#
#     depth_xy = depth_frame.get_distance(int(x + x_min_offset), int(y + y_min_offset))
#     shape_point_xyz = np.array(rs.rs2_deproject_pixel_to_point(depth_intrinsics, [int(x + x_min_offset), int(y + y_min_offset)], depth_xy))
#     print(np.dot(shape_point_xyz, 1000))
#
#     # putting shape name at center of each shape
#     if len(approx) == 3:
#         cv2.putText(img, 'Triangle', (x, y),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#
#     elif len(approx) == 4:
#         cv2.putText(img, 'Quadrilateral', (x, y),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#
#     elif len(approx) == 5:
#         cv2.putText(img, 'Pentagon', (x, y),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#
#     elif len(approx) == 6:
#         cv2.putText(img, 'Hexagon', (x, y),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#
#     elif len(approx) == 8:
#         cv2.putText(img, 'Octagon', (x, y),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#
#     else:
#         cv2.putText(img, 'ellipse', (x, y),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#
#     # displaying the image after drawing contours
#     cv2.imshow('shapes', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# list for storing names of shapes
for contour in contours:

    # here we are ignoring first counter because
    # findcontour function detects whole image as shape
    if i == 0:
        i = 1
        continue

    # cv2.approxPloyDP() function to approximate the shape
    approx = cv2.approxPolyDP(
        contour, 0.05 * cv2.arcLength(contour, True), True)
    print(len(approx))

    # using drawContours() function
    cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

    # finding the center point of shape
    M = cv2.moments(contour)
    #print(M['m00'])
    if M['m00'] != 0.0:
        x = int(M['m10'] / M['m00'])
        y = int(M['m01'] / M['m00'])
    print("x, y = ", x + x_min_offset, y + y_min_offset)

    depth_xy = depth_frame.get_distance(int(x + x_min_offset), int(y + y_min_offset))
    shape_point_xyz = np.array(rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(x + x_min_offset), int(y + y_min_offset)], depth_xy))
    print(np.dot(shape_point_xyz, 1000))

    # # putting shape name at center of each shape
    # if len(approx) == 3:
    #     cv2.putText(img, 'Triangle', (x, y),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    #
    # elif len(approx) == 4:
    #     cv2.putText(img, 'Quadrilateral', (x, y),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    #
    # elif len(approx) == 5:
    #     cv2.putText(img, 'Pentagon', (x, y),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    #
    # elif len(approx) == 6:
    #     cv2.putText(img, 'Hexagon', (x, y),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    #
    # elif len(approx) == 8:
    #     cv2.putText(img, 'Octagon', (x, y),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    #
    # else:
    #     cv2.putText(img, 'ellipse', (x, y),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # # displaying the image after drawing contours
    # cv2.imshow('shapes', img)
    offsetpickx = -10
    offsetpicky = -2
    #tag.pose_t[2] = tags_point_xyz_m[i][2]  # replace the depth component of the pose estimation
    #pose_rot = np.identity(3)
    pose_t = shape_point_xyz  # replace the depth component of the pose estimation
    pose_trans = np.dot(np.array(pose_t), 1000)
    T_c_cube = SE3.Rt(pose_rot, pose_trans)
    print(T_c_cube)
    T_b_cube = T_b_c * T_c_cube
    print("T_b_cube = ")
    print(T_b_cube)
    print(T_b_cube.x, T_b_cube.y, T_b_cube.z)  # x, y,and z elements of the position vector
    pos_x = T_b_cube.x
    pos_y = T_b_cube.y
    pos_z = 38 #T_b_cube.z
    #arm.set_position(x=pos_x, y=pos_y, z=pos_z + 50, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # go to above cube position
    print(pos_x, pos_y, pos_z)  # x, y,and z elements of the position vector
    arm.set_position(x=pos_x + offsetpickx, y=pos_y + offsetpicky, z=pos_z + 150, roll=-180, pitch=0, yaw=0, speed=100,wait=True)  # go to above cube position
    arm.open_lite6_gripper()  # activate gripper
    arm.set_position(x=pos_x + offsetpickx, y=pos_y + offsetpicky, z=pos_z, roll=-180, pitch=0, yaw=0, speed=100, wait=True) # go to cube position
    arm.set_pause_time(1)
    arm.set_position(x=pos_x + offsetpickx, y=pos_y + offsetpicky, z=pos_z + 150, roll=-180, pitch=0, yaw=0, speed=100, wait=True) # go to above cube position

    offsetx = 0 # global x offset
    offsety = 0 # global y offset
    base = 175 - 1 # height to initially move the block to above the cube
    baseoffset = 50 # height to go above the cube
    yaw = 7 # angle in deg to wiggle the yaw
    miniset = 3 # depth to plunge
    wiggleoffset = 4 # distance to move left or right


    if len(approx) == 4:
        x4 = 126
        y4 = 143
        arm.set_position(x=x4 + offsetx, y=y4 + offsety, z=base + baseoffset, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # move to above cube
        arm.set_position(x=x4 + offsetx, y=y4 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # Lower block to be touching cube
        arm.set_pause_time(0.1)
        arm.set_position(x=x4 + offsetx, y=y4 + offsety, z=base, roll=-180, pitch=0, yaw=yaw, speed=50, wait=True) # roll the block some
        arm.set_position(x=x4 + offsetx, y=y4 + offsety, z=base, roll=-180, pitch=0, yaw=-yaw, speed=50, wait=True) # roll the other way
        arm.set_position(x=x4 + offsetx, y=y4 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # go back to normal orientation
        arm.set_position(x=x4 + offsetx + wiggleoffset, y=y4 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # move forwards in x
        arm.set_position(x=x4 + offsetx - wiggleoffset, y=y4 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # move back in x
        arm.set_position(x=x4 + offsetx, y=y4 + offsety + wiggleoffset, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # move forwards in y
        arm.set_position(x=x4 + offsetx, y=y4 + offsety - wiggleoffset, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # move back in y
        arm.set_position(x=x4 + offsetx, y=y4 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # move back to original position
        arm.set_position(x=x4 + offsetx, y=y4 + offsety, z=base - miniset, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # lower block into hole

    elif len(approx) == 5:
        x5 = 156
        y5 = 204.2
        arm.set_position(x=x5 + offsetx, y=y5 + offsety, z=base + baseoffset, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx, y=y5 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_pause_time(0.1)
        arm.set_position(x=x5 + offsetx, y=y5 + offsety, z=base, roll=-180, pitch=0, yaw=yaw, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx, y=y5 + offsety, z=base, roll=-180, pitch=0, yaw=-yaw, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx, y=y5 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx + wiggleoffset, y=y5 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx - wiggleoffset, y=y5 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx, y=y5 + offsety + wiggleoffset, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx, y=y5 + offsety - wiggleoffset, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx, y=y5 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x5 + offsetx, y=y5 + offsety, z=base - miniset, roll=-180, pitch=0, yaw=0, speed=50, wait=True)

    elif len(approx) == 6:
        x6 = 189.1
        y6 = 144
        arm.set_position(x=x6 + offsetx, y=y6 + offsety, z=base + baseoffset, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx, y=y6 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_pause_time(0.1)
        arm.set_position(x=x6 + offsetx, y=y6 + offsety, z=base, roll=-180, pitch=0, yaw=yaw, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx, y=y6 + offsety, z=base, roll=-180, pitch=0, yaw=-yaw, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx, y=y6 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx + wiggleoffset, y=y6 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx - wiggleoffset, y=y6 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx, y=y6 + offsety + wiggleoffset, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx, y=y6 + offsety - wiggleoffset, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx, y=y6 + offsety, z=base, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
        arm.set_position(x=x6 + offsetx, y=y6 + offsety, z=base-miniset, roll=-180, pitch=0, yaw=0, speed=50, wait=True)


    #arm.set_position(x=150, y=0, z=50 + height + offset, roll=-180, pitch=0, yaw=0, speed=50,wait=True)  # go to above placement position
    #arm.set_position(x=150, y=0, z=height + offset, roll=-180, pitch=0, yaw=0, speed=50,wait=True)  # go to placement position
    arm.close_lite6_gripper()  # deactivate gripper
    arm.set_pause_time(1)
    arm.set_position(x=170, y=180, z=300, roll=-180, pitch=0, yaw=0, speed=50,wait=True)  # go to above placement position
    arm.move_gohome(wait=True)
    i = i + 1  # tags index increase by one
    offset = offset + height
    #offset = offset



#---------------------------------------------------------------------------------------------------------------
#i = 0  # tags index set to zero
#offset = 0
#height = 40
# -- find their pose
#print("START LOOP")
#for tag in tags_cube:
#    pose_rot = np.array(tag.pose_R)
#    tag.pose_t[2] = tags_point_xyz_m[i][2]  # replace the depth component of the pose estimation
#    pose_trans = np.dot(np.array(tag.pose_t), 1000)
#    T_c_cube = SE3.Rt(pose_rot, pose_trans)
#    T_b_cube = T_b_c * T_c_cube
#    print(T_b_cube.x, T_b_cube.y, T_b_cube.z)  # x, y,and z elements of the position vector
#    arm.set_position(x=T_b_cube.x, y=T_b_cube.y, z=T_b_cube.z + 50, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # go to above cube position
#    arm.open_lite6_gripper()  # activate gripper
#    arm.set_position(x=T_b_cube.x, y=T_b_cube.y, z=T_b_cube.z, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # go to cube position
#    arm.set_pause_time(1)
#    arm.set_position(x=T_b_cube.x, y=T_b_cube.y, z=T_b_cube.z + 50, roll=-180, pitch=0, yaw=0, speed=50, wait=True) # go to above cube position
#    arm.set_position(x=150, y=0, z=50 + height + offset, roll=-180, pitch=0, yaw=0, speed=50,wait=True)  # go to above placement position
#    arm.set_position(x=150, y=0, z=height + offset, roll=-180, pitch=0, yaw=0, speed=50,wait=True)  # go to placement position
#    arm.close_lite6_gripper()  # deactivate gripper
#    arm.set_pause_time(1)
#    arm.set_position(x=150, y=0, z=100 + height + offset, roll=-180, pitch=0, yaw=0, speed=50,wait=True)  # go to above placement position
#    arm.move_gohome(wait=True)
#    i = i + 1  # tags index increase by one
#    #offset = offset + height
#    offset = offset
#print("END LOOP")
# -- send the arm back to the home position
arm.move_gohome(wait=True)
print(arm.get_position())

# -- disconnect the arm
arm.disconnect()
