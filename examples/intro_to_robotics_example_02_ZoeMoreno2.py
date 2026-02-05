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
time_pause = 2

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=95, y=-250, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=95, y=-250, z=90, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=95, y=-250, z=150, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=190, y=70, z=80, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=190, y=70, z=30, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=190, y=70, z=80, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=95, y=-250, z=110, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=95, y=-250, z=59, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=95, y=-250, z=110, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=280, y=-30, z=80, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=280, y=-30, z=30, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=280, y=-30, z=80, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=90, y=-235, z=80, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=90, y=-235, z=29, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=90, y=-235, z=80, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)


# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=220, y=-105, z=80, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=220, y=-105, z=30, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

# send the arm to the pose (x,y,z,roll,pitch,yaw) with speed of 50 mm/s
arm.set_position(x=220, y=-105, z=80, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_pause_time(time_pause)

arm.move_gohome(wait=True)
arm.disconnect()
