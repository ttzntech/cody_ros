#
# Created on Tue Aug 13 2024
#
# Author: Joonchen Liau
# Email: liaojuncheng123@foxmail.com
#
# Copyright (c) 2024 Tiante Intelligent Technology
#

#! /usr/bin/env python3
import sys, tty, termios, select

import rospy
from cody_msgs.msg import MoveCtrl 


RESET = "\033[0m"

BOLD = "\033[1m"
UNDERLINE = "\033[4m"

RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"

logo = f"""{BOLD}{CYAN}
          _______ _______ _______   _                _   _           _ 
         |__   __|__   __|___  / \ | |              (_) | |         (_)
            | |     | |     / /|  \| | __      _____ _  | |__   __ _ _ 
            | |     | |    / / | . ` | \ \ /\ / / _ \ | | '_ \ / _` | |
            | |     | |   / /__| |\  |  \ V  V /  __/ | | | | | (_| | |
            |_|     |_|  /_____|_| \_|   \_/\_/ \___|_| |_| |_|\__,_|_|
{MAGENTA}
         author: Joonchen Liau        email:liaojuncheng123@foxmail.com                                                                       
{RESET}
"""

def bound(val, low, up):
    if val < low:
        return low
    elif val > up:
        return up
    else:
        return val

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        # Check if there's data available to read
        rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
        if rlist:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class Robot:
    def __init__(self, name='teleop_node', rate=50, 
                 v_inc=0.1, s_inc=3, log=True):
        rospy.init_node(name)
        self.cmd_pub = rospy.Publisher("/cmd_vel", MoveCtrl, queue_size=1)
        self.cmd_msg = MoveCtrl()
        self.loop_rate = rospy.Rate(rate)
        self.v_inc = v_inc      # speed increment 
        self.s_inc = s_inc      # corner (steering angle) increment
        self.log = log

    def publish(self):
        self.cmd_pub.publish(self.cmd_msg)
    
    def stop(self):
        self.cmd_msg.speed = 0
        self.cmd_msg.corner = 0

    def run(self):
        print(logo)
        print("########################### Teleop control begin #############################")
        print(f"Teleop control started. Use arrow keys to control the robot. Press {RED}'q' to quit.{RESET}")
        print(f"Use {RED}'w'/'s' for forward/backward, 'a'/'d' for left/right turn and 'space' for stop.{RESET}")
        print('\n')
        try:
            while not rospy.is_shutdown():
                key = get_key()
                if key == 'q':
                    break
                if key == 'w':
                    self.cmd_msg.speed += self.v_inc
                if key == 's':
                    self.cmd_msg.speed -= self.v_inc 
                if key == 'a':
                    self.cmd_msg.corner -= self.s_inc 
                if key == 'd':
                    self.cmd_msg.corner += self.s_inc 
                if key == ' ':
                    self.cmd_msg.speed = 0

                self.cmd_msg.speed = bound(self.cmd_msg.speed, -2, 2)
                self.cmd_msg.corner = bound(self.cmd_msg.corner, -30, 30)

                if self.log:
                    print(f"            speed: {self.cmd_msg.speed:.4f} m/s\t corner: {self.cmd_msg.corner:.4f} degree", end='\r')
                
                self.publish()
                self.loop_rate.sleep()
        finally:
            # stop the vehicle
            self.stop()
            self.publish()
            print("########################## Teleop control stopped ############################")


if __name__ == "__main__":
    robot = Robot()
    robot.run()