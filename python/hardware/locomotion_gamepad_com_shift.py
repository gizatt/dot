#!/usr/bin/env python

from __future__ import print_function

import argparse
from copy import deepcopy
import sys
import time
from inputs import get_gamepad
import numpy as np
import rospy
import logging
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
    args = parser.parse_args()
    rospy.init_node('locomotion_gamepad_com_commander', anonymous=True)

    print(
"""
Current mode:
- Use left stick for XY COM shifting.
- Use right stick for yaw + pitch.
- Use analog shoulder buttons for roll.
- Use up/down on dpad for changing z height.
"""
    )

    def scale_axis(val, bounds):
        # Val on [-1, 1]; bounds is tuple of [lb, ub]
        scale = (val + 1)/2.
        return bounds[0] * (1. - scale) + scale * bounds[1]

    x_key = 'ABS_Y' # left forward/back
    x_range = [0.1, -0.1]
    x_max = 32767.

    y_key = 'ABS_X' # left left/right
    y_range = [0.05, -0.05]
    y_max = 32767.

    pitch_key = 'ABS_RY' # right forward/backl
    pitch_range = [-0.25, 0.25]
    pitch_max = 32767.
    yaw_key = 'ABS_RX' # right left/right
    yaw_range = [0.4, -0.4]
    yaw_max = 32767.
    
    roll_neg_key = "ABS_Z"
    roll_pos_key = "ABS_RZ"
    roll_range = [-0.5, 0.5]
    roll_max = 255.

    # z is dpad y axis
    z_key = "ABS_HAT0Y"
    z_range = [0.05, 0.15]
    z_step = 0.005

    xyzrpy = np.zeros(6)
    xyzrpy[2] = 0.125

    last_pub = time.time()
    pub_period = 0.05
    while True:
        events = get_gamepad()
        for event in events:
            if event.ev_type == "Absolute":
                if event.code == x_key:
                    xyzrpy[0] = scale_axis(event.state / x_max, x_range)
                elif event.code == y_key:
                    xyzrpy[1] = scale_axis(event.state / y_max, y_range)
                elif event.code == z_key:
                    xyzrpy[2] = min(max(z_range[0], xyzrpy[2] - z_step * event.state), z_range[1])
                elif event.code == roll_neg_key:
                    xyzrpy[3] = scale_axis(-event.state / roll_max, roll_range)
                elif event.code == roll_pos_key:
                    xyzrpy[3] = scale_axis(event.state / roll_max, roll_range)
                elif event.code == pitch_key:
                    xyzrpy[4] = scale_axis(event.state / pitch_max, pitch_range)
                elif event.code == yaw_key:
                    xyzrpy[5] = scale_axis(event.state / yaw_max, yaw_range)
        
        if time.time() - last_pub > pub_period:
            last_pub = time.time()
            print("XYZRPY: ", xyzrpy)

            move_com_relative(xyzrpy[:3], xyzrpy[3:])