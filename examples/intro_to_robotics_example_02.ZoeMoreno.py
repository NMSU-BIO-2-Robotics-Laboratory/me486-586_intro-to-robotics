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
time_pause = 5

# open the gripper
arm.open_lite6_gripper()
arm.set_pause_time(1)

arm.set_servo_angle(angle=[-69.791588, 23.423832, 53.284273, -0.51486, 29.690788, -69.147297], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-69.8655, 36.193916, 53.301175, -1.102142, 16.969979, -68.689618], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-69.794453, 23.437641, 53.303868, -0.501395, 29.724363, -69.153198], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[19.486868, 27.705718, 33.248111, 0.750174, 5.510765, 18.789349], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[19.805948, 43.784804, 37.845868, 4.632708, -5.960766, 15.129695], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# close the gripper
arm.close_lite6_gripper()
arm.set_pause_time(2)
# stop the gripper operation (it is not necessary for the vacuum)
arm.stop_lite6_gripper()
arm.set_pause_time(2)

arm.set_servo_angle(angle=[19.486868, 27.705775, 33.248168, 0.750174, 5.510708, 18.789349], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# open the gripper
arm.open_lite6_gripper()
arm.set_pause_time(1)

arm.set_servo_angle(angle=[-69.841321, 31.63999, 52.812156, -0.834284, 21.032708, -68.912098], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-69.90383, 43.701897, 55.019074, -1.813297, 11.185912, -68.047848], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-69.841321, 31.640047, 52.812156, -0.834284, 21.032708], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-6.729275, 41.051968, 58.058845, -0.204317, 17.015643, -6.415981], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-6.744458, 53.075487, 61.930263, -0.663084, 8.86635, -6.038746], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# close the gripper
arm.close_lite6_gripper()
arm.set_pause_time(2)
# stop the gripper operation (it is not necessary for the vacuum)
arm.stop_lite6_gripper()
arm.set_pause_time(2)

arm.set_servo_angle(angle=[-6.729275, 41.051968, 58.058845, -0.204317, 17.015643, -6.415924], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# open the gripper
arm.open_lite6_gripper()
arm.set_pause_time(1)

arm.set_servo_angle(angle=[-69.773368, 35.839312, 48.640966, -1.515588, 12.668211, -68.200885], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-69.840117, 49.309435, 52.799551, -6.392547, 3.399244, -63.441153], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-69.773368, 35.839312, 48.641023, -1.515588, 12.668211, -68.200943], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.set_servo_angle(angle=[-26.274068, 34.530104, 45.917411, -0.819387, 11.386677, -25.384609],  speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)
 
arm.set_servo_angle(angle=[-26.310623, 48.138252, 50.034915, -6.143998, 1.934649, -20.161353], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# close the gripper
arm.close_lite6_gripper()
arm.set_pause_time(2)
# stop the gripper operation (it is not necessary for the vacuum)
arm.stop_lite6_gripper()
arm.set_pause_time(2)

arm.set_servo_angle(angle=[-26.274068, 34.530104, 45.917411, -0.819387, 11.386677, -25.384609], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.move_gohome(wait=True)
arm.disconnect()
