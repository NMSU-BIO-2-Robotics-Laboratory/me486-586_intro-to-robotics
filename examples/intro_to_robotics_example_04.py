#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
# Modified by Dr. Haghshenas-Jaryani, mahdihj@nmsu.edu, to be use in ME486/586 intro-to-robotics at NMSU

"""
Description: Move Arc line(linear arc motion)
"""
import os
import sys
from xarm.wrapper import XArmAPI


#######################################################
path_to_config = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')

if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read(path_to_config + 'robot.conf')
        ip = parser.get('Lite6', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.move_gohome(wait=True)

paths = [
    [150, 0, 150, -180, 0, 0],
    [150, 200, 250, -180, 0, 0],
    [300, 200, 150, -180, 0, 0],
    [300, -200, 250, -180, 0, 0],
    [150, -200, 150, -180, 0, 0],
    [150, 0, 250, -180, 0, 0],
    [150, 200, 300, -180, 0, 0],
    [300, 200, 250, -180, 0, 0],
    [300, -200, 300, -180, 0, 0],
    [150, -200, 250, -180, 0, 0],
    [150, 0, 300, -180, 0, 0],
]

arm.set_position(*paths[0], wait=True)
_, angles = arm.get_servo_angle()
arm.set_pause_time(0.2)


def move():
    ret = arm.set_servo_angle(angle=angles, speed=30, wait=False)
    if ret < 0:
        print('set_servo_angle, ret={}'.format(ret))
        return -1
    for path in paths:
        ret = arm.set_position(*path[:6], radius=0, wait=False, speed=50)
        if ret < 0:
            print('set_position, ret={}'.format(ret))
            return -1
    return 0

# the main part of code


for i in range(1):
    if move() != 0:
        break

arm.move_gohome(wait=True)
arm.disconnect()
