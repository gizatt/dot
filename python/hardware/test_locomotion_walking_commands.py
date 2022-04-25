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
from dot_msgs.msg import LocomotionServerStatus
from dot_msgs.srv import (
    MoveCom, MoveComRequest,
    MoveFoot, MoveFootRequest
)

def np_from_vector3(vec3):
    return np.array([vec3.x, vec3.y, vec3.z])
def vector3_from_np(vec):
    return Vector3(vec[0], vec[1], vec[2])

class Walker():
    # Manages walking state machine: cycle through feet in LR/LF/RR/RF order,
    # shifting COM to center of stance feet, picking up foot, moving it forward a few cm,
    # and putting it down.

    def __init__(self, com_height=0.15, com_slew_size=0.25, com_shift_frac=0.1, foot_slew_size=0.05, forward_step_size=0.1, foot_lift_amount=0.02):
        self.curr_robot_state = None
        self.curr_feet_positions = None
        self.curr_stance_status = None
        rospy.Subscriber("/robot_state", JointState, self.handle_robot_state)
        rospy.Subscriber("/locomotion_server_status", LocomotionServerStatus, self.handle_locomotion_status)
        
        rospy.sleep(0.25)
        while (self.curr_robot_state) is None:
            rospy.loginfo("Waiting for robot state...")
            rospy.sleep(0.1)
        while (self.curr_feet_positions) is None:
            rospy.loginfo("Waiting for feet positions...")
            rospy.sleep(0.1)
        rospy.wait_for_service('/locomotion_server_move_com')
        rospy.wait_for_service('/locomotion_server_move_foot')

        self.foot_ordering = [
            "left_rear_leg", "left_front_leg", "right_rear_leg", "right_front_leg"
        ]
        self.nominal_foot_offsets = {
            "left_front_leg": np.array([0.12, 0.1, 0.038]),
            "right_front_leg": np.array([0.12, -0.1, 0.038]),
            "left_rear_leg": np.array([-0.175, 0.1, 0.038]),
            "right_rear_leg": np.array([-0.175, -0.1, 0.038]),
        }
        self.com_shift_frac = com_shift_frac
        self.com_slew_size = com_slew_size
        self.foot_slew_size = foot_slew_size
        self.forward_step_size = forward_step_size
        self.foot_lift_amount = foot_lift_amount
        self.curr_foot_k = 0
        self.com_height = com_height
        self.curr_commanded_com = None
        self.curr_commanded_foot_pos = None

    def handle_robot_state(self, msg):
        self.curr_robot_state = np.array(msg.position[:])

    def handle_locomotion_status(self, msg):
        self.curr_feet_positions = {msg.leg_names[k]: np_from_vector3(msg.feet_positions[k]) for k in range(4)}
        self.curr_stance_status = {msg.leg_names[k]: msg.is_stance[k] for k in range(4)}

    def do_step(self, direction=np.array([1., 0.])):
        # Curr state should be all feet on the ground.
        if not all(self.curr_stance_status.values()):
            rospy.logerr("Not all feet on ground?")
            return False

        # Pick foot to advance.
        target_foot = self.foot_ordering[self.curr_foot_k]
        self.curr_foot_k = (self.curr_foot_k + 1) % 4

        # Move COM towards center of the stance feet.
        com_target = np.stack([
            v for k, v in self.curr_feet_positions.items() if k != target_foot
        ]).mean(axis=0)
        rospy.loginfo("Feet pos: %s", self.curr_feet_positions)
        rospy.loginfo("COM target: %s", com_target)
        # Keep original com height
        com_target[2] = self.com_height # self.curr_robot_state[2]
        com_target = com_target * (self.com_shift_frac) + self.curr_robot_state[:3] * (1. - self.com_shift_frac)
        if not self.move_com_to(com_target):
            rospy.logwarn("Couldn't move COM all the way.")
            # Keep going in case we can still make it partway.
        
        # Pick up foot
        original_foot_pos = deepcopy(self.curr_feet_positions[target_foot])
        lift_pos = original_foot_pos + np.array([0., 0., self.foot_lift_amount])
        if not self.move_foot_to(target_foot, lift_pos):
            rospy.logwarn("Couldn't lift foot completely.")
            # Keep going in case we can still make it partway.

        
        # Move foot over new position
        direction = direction / np.sum(direction)
        reach_pos = lift_pos + np.r_[direction, 0.]*self.forward_step_size
        if not self.move_foot_to(target_foot, reach_pos):
            rospy.logwarn("Couldn't reach foot completely.")
            # Keep going in case we can still make it partway.

        # Drop foot
        drop_pos = reach_pos + np.array([0., 0., -self.foot_lift_amount])
        if not self.move_foot_to(target_foot, drop_pos, end_in_support=True):
            rospy.logerr("Couldn't drop foot completely.")
            return False

        # Recenter COM
        com_target = np.stack([
            v for k, v in self.curr_feet_positions.items()
        ]).mean(axis=0)
        com_target[2] = self.com_height
        if not self.move_com_to(com_target):
            rospy.logwarn("Couldn't move COM all the way.")
            # Keep going in case we can still make it partway.

        # Done!
        return True
        
    def wait_for_new_state(self):
        self.handle_robot_state(rospy.wait_for_message('/robot_state', JointState))
    def wait_for_new_status(self):
        self.handle_locomotion_status(rospy.wait_for_message('/locomotion_server_status', LocomotionServerStatus))

    def move_foot_to(self, foot_name, foot_target, end_in_support=False):
        # Interpolate foot position to the given foot target. If end_in_support,
        # send one more message at the end setting to foot to the final position
        # in support mode.
        move_com_srv = rospy.ServiceProxy('/locomotion_server_move_foot', MoveFoot)
        def send(foot_name, desired_xyz, end_this_in_support):
            try:
                cmd = MoveFootRequest(
                    leg_name=foot_name,
                    desired_xyz=vector3_from_np(desired_xyz),
                    end_in_support=Bool(end_this_in_support)
                )
                resp1 = move_com_srv(cmd)
                if not resp1.success:
                    rospy.logerr("Move COM error: ", resp1.info)
                    return False
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                return False
            return True
    
        self.wait_for_new_status()
        self.curr_commanded_foot_pos = self.curr_feet_positions[foot_name]
        while np.linalg.norm(self.curr_commanded_foot_pos - foot_target) >= 1E-3:
            self.curr_commanded_foot_pos += np.clip(
                foot_target - self.curr_commanded_foot_pos,
                -self.foot_slew_size, self.foot_slew_size)
            if not send(foot_name, self.curr_commanded_foot_pos, False):
                return False
            rospy.sleep(0.05)
        
        if end_in_support:
            return send(foot_name, self.curr_commanded_foot_pos, True)
        return True


    def move_com_to(self, com_target):
        # Interpolate com position to the given target.
        move_com_srv = rospy.ServiceProxy('/locomotion_server_move_com', MoveCom)
        self.wait_for_new_state()
        self.curr_commanded_com = self.curr_robot_state[:3]
        while np.linalg.norm(self.curr_commanded_com - com_target) >= 1E-3:
            self.curr_commanded_com += np.clip(com_target - self.curr_commanded_com,
                                               -self.com_slew_size, self.com_slew_size)
            try:
                cmd = MoveComRequest(desired_xyz=vector3_from_np(self.curr_commanded_com))
                resp1 = move_com_srv(cmd)
                if not resp1.success:
                    rospy.logerr("Move COM error: ", resp1.info)
                    return False
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                return False
            rospy.sleep(0.05)
        return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Issue locomotion ROS actions.')
    parser.add_argument('-n_steps', type=float, default=1)
    parser.add_argument('-dir', nargs=2, type=float, metavar=('x', 'y'), default=[1., 0.])
    args = parser.parse_args()
    
    rospy.init_node('locomotion_commander', anonymous=True)
    walker = Walker()

    step_k = 0
    while not rospy.is_shutdown() and step_k < args.n_steps:
        walker.do_step(np.array(args.dir))
        step_k += 1
            


