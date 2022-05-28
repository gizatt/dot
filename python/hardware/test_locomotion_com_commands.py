#!/usr/bin/env python

from __future__ import print_function

import argparse
from copy import deepcopy
import sys
import time
import numpy as np
import rospy
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from dot_msgs.srv import (
    MoveCom, MoveComRequest,
    MoveFoot, MoveFootRequest
)

def move_com_relative(com_target_xyz, com_target_rpy):
    rospy.wait_for_service('/locomotion_server_move_com')
    try:
        move_com_srv = rospy.ServiceProxy('/locomotion_server_move_com', MoveCom)
        cmd = MoveComRequest(desired_xyz=Vector3(*com_target_xyz), desired_rpy=Vector3(*com_target_rpy))
        resp1 = move_com_srv(cmd)
        return resp1.success, resp1.info
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Issue locomotion ROS actions.')
    parser.add_argument('-com_rel', nargs=3, type=float, metavar=('x', 'y', 'z'), default=None)
    parser.add_argument('-com_rel_full', nargs=6, type=float, metavar=('x', 'y', 'z', 'r', 'p', 'y'), default=None)
    parser.add_argument('-com_spiral', nargs=9, type=float, metavar=('tx', 'ty', 'tz', 'ax', 'ay', 'az', 'px', 'py', 'pz'), default=None)
    parser.add_argument('-step', type=float, default=0.005)
    args = parser.parse_args()
    
    rospy.init_node('locomotion_commander', anonymous=True)
    joint_state = rospy.wait_for_message('/robot_state', JointState)
    com = np.array(joint_state.position[:3])
    com_full = np.array(joint_state.position[:6])

    if args.com_rel:
        com_target = np.array(args.com_rel)
        print("Doing COM rel command to ", com_target, " from ", com)
        while np.linalg.norm(com - com_target) >= 1E-3:
            print(com_target - com)
            com = com + np.clip(com_target - com, -args.step, args.step)
            print("Com: ", com)
            print(move_com_relative(com, np.zeros(3)))
    elif args.com_rel_full:
        com_target_full = np.array(args.com_rel_full)
        print("Doing COM rel command to ", com_target_full, " from ", com_full)
        while np.linalg.norm(com_full - com_target_full) >= 1E-3:
            print(com_target_full - com_full)
            com_full = com_full + np.clip(com_target_full - com_full, -args.step, args.step)
            print("Com: ", com_full)
            print(move_com_relative(com_full[:3], com_full[3:]))
    elif args.com_spiral:
        com_center = deepcopy(com)
        period = np.array(args.com_spiral[:3])
        amplitude = np.array(args.com_spiral[3:6])
        phase = np.array(args.com_spiral[-3:])
        print("Spiraling around ", com, " with amplitude ", amplitude, " and period ", period, " and phase ", phase)
        t0 = time.time()
        while (1):
            t = time.time() - t0
            theta = (np.ones(3) * (t * 2 * np.pi)) / period
            com_target = com_center + np.sin(theta + phase)*amplitude
            # Step towards that target
            com = com + np.clip(com_target - com, -args.step, args.step)
            print("Com: ", com)
            print(move_com_relative(com, np.zeros(3)))
            


