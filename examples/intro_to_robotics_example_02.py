#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
# Modified by Dr. Haghshenas-Jaryani, mahdihj@nmsu.edu, to be use in ME486/586 intro-to-robotics at NMSU

"""
Description: Move Joint
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
        parser.read(path_to_config + '/robot.conf')
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

speed = 30
arm.set_servo_angle(angle=[90, 0, 0, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(angle=[90, 0, +60, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(angle=[90, -30, +60, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(angle=[0, -30, +60, +45, 0, +90], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(angle=[0, 0, +60, 0, -45, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)

arm.set_servo_angle(servo_id=1, angle=90, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(servo_id=3, angle=+60, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(servo_id=2, angle=-30, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(servo_id=1, angle=0, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(servo_id=2, angle=0, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)
arm.set_servo_angle(servo_id=3, angle=0, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(5)


arm.move_gohome(wait=True)
arm.disconnect()
