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

import sys
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
        parser.read('/config/robot.conf')
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

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=150, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)

# get pose information
print(arm.get_position(), arm.get_position(is_radian=True))

# open the gripper
arm.open_lite6_gripper()
arm.set_pause_time(5)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=250, y=200, z=250, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))

# close the gripper
arm.close_lite6_gripper()
arm.set_pause_time(5)
# stop the gripper operation (it is not necessary for the vacuum)
arm.stop_lite6_gripper()
arm.set_pause_time(2)

# send the arm to the pose (x(mm),y(mm),z(mm),roll(deg),pitch(deg),yaw(deg)) with speed of 50 mm/s
arm.set_position(x=150, y=0, z=250, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_pause_time(5)

# send the arm back to the home position
arm.move_gohome(wait=True)
print(arm.get_position())

# disconnect the arm
arm.disconnect()
