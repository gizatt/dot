import argparse
from copy import deepcopy
from leg_controller import *
from ik import single_leg_forward_kin
import yaml
import time
from scipy.optimize import linprog
import scipy
import scipy.spatial
import numpy as np

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from dot_msgs.msg import LocomotionServerStatus
from dot_msgs.srv import (
    MoveCom, MoveComResponse,
    MoveFoot, MoveFootResponse
)


'''
Provides a locomotion management service.

1) Publishes current robot state, including a global
estimate of the estimated COM position based on
odometry estimates. Tracks which feet are currently being
used for support, and keeps track of the support polygon
(assuming flat ground).

2) Provides services for moving the COM and feet in ways
that check for stability and IK feasibility:

a) MoveCom: Moves COM to desired (world) position, or rejects
if it moves the COM out of the support polygon or is IK-infeasible
or violates joint constraints.
b) MoveFoot: Marks target foot as *not* a support foot and moves it
to the target (world) location, keeping COM fixed. Rejects if removing
the foot from support moves COM out of the support polygon, or if it's
IK infeasible or violates joint constraints to move the foot to that position.
If a flag is set, makes the foot back into a support foot at the end of motion.

'''

def in_hull(com, points, contract_amount=0.):
    """
    com is (M,)
    points is (M, N)
    Based on https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
    Modified for contraction test
    """
    n_points = points.shape[1]
    n_dim = points.shape[0]

    # Contract points toward center by desired factor
    center = points.mean(axis=1)
    points = ((points * (1. - contract_amount)).T + center * contract_amount).T

    # Just a feasibility check; no cost
    c = np.zeros(n_points)
    # points combined weighted by x = com
    # \sum x = 1 (final row)
    A = np.r_[points,np.ones((1,n_points))]
    b = np.r_[com, np.ones(1)]
    lp = linprog(c, A_eq=A, b_eq=b)
    return lp.success


def do_ik(ee_target, hip_1_in_root_frame, hip_flip=1.):
    angles = single_leg_forward_kin(
        ee_target=ee_target,
        hip_1_in_root_frame=hip_1_in_root_frame,
        hip_2_in_hip_1_frame=np.array([0.025, hip_flip*0.043, 0.0]),
        hip_length=0.11,
        shin_length=0.11
    )
    if angles is False:
        return False
    angles[0] *= hip_flip
    return angles


class LocomotionManager():
    state_publish_period = 0.1
    status_publish_period = 0.1
    command_publish_period = 1.0

    def __init__(self, z0=0.125):
        with open("servo_config.yaml") as f:
            self.servo_configs = yaml.load(f)

        com_q0 = np.array([0., 0., z0, 0., 0., 0.])

        # hip_1_in_root_frame for each foot.
        # +x forward, +y right. COM position is taken
        # to be the dead center of the two hips.
        self.hip_positions = {
            "left_front_leg": [np.array([0.12, 0.05, 0.038]), 1.],
            "right_front_leg": [np.array([0.12, -0.05, 0.038]), -1.],
            "left_rear_leg": [np.array([-0.175, 0.05, 0.038]), 1.],
            "right_rear_leg": [np.array([-0.175, -0.05, 0.038]), -1.]
        }
        # COM offset
        #for k in self.hip_positions.keys():
        #    self.hip_positions[k][0] -= np.array([0.05, 0., 0.])
        self.leg_names = self.hip_positions.keys()

        # Build state names for the state publisher.
        self.state_names = \
            ["com_x", "com_y", "com_z", "com_r", "com_p", "com_y"] + \
            ["" for k in range(12)]
        for f in self.leg_names:
            for key in ["hip_abduct", "hip_pitch", "knee_pitch"]:
                self.state_names[6 + self.servo_configs[f][key + "_info"]["pose_ind"]] = f[:-4] + key
        
        # Setup initial state with feet on ground.
        initial_feet_positions = {
            f: np.r_[v[0][:2], 0.] + np.array([0., v[1]*0.05, 0.]) for f, v in self.hip_positions.items()
        }
        q0 = self.do_ik_for_feet_and_com(initial_feet_positions, com_q0)
        assert q0 is not False, "Infeasible initial foot positions."
        self.stance_status = {f: True for f in self.leg_names}
        self.feet_positions = initial_feet_positions
        self.com_q = np.array(com_q0)
        self.joint_q = np.array(q0)
        assert self.com_q.shape == (6,)
        assert self.joint_q.shape == (12,)

        # Setup state and status publishers.
        self.state_publish_seq = 0
        self.status_publish_seq = 0
        self.state_pub = rospy.Publisher('robot_state', JointState, queue_size=1)
        self.state_publish_timer = rospy.Timer(
            rospy.Duration(self.state_publish_period), self.publish_state
        )
        self.status_pub = rospy.Publisher('locomotion_server_status', LocomotionServerStatus, queue_size=1)
        self.status_publish_timer = rospy.Timer(
            rospy.Duration(self.status_publish_period), self.publish_status
        )

        # Setup movement services.
        self.move_com_srv = rospy.Service('locomotion_server_move_com', MoveCom, self.handle_move_com)
        self.move_foot_srv = rospy.Service('locomotion_server_move_foot', MoveFoot, self.handle_move_foot)

        # Finally create leg interface driver and try to send initial state.
        self.leg_interface = HardwareInterface(self.joint_q, self.servo_configs)
        assert self.leg_interface.send_pose(self.joint_q, allow_projection=False)
        self.command_timer = rospy.Timer(
            rospy.Duration(self.command_publish_period), self.publish_command
        )

    def do_ik_for_feet_and_com(self, feet_positions, com_xyz, com_rpy=np.zeros(3)):
        q = np.zeros(12)
        # Hips should be in a plane that intersects com_xyz, and is rotated
        # by this RPY.
        R = sp.spatial.transform.Rotation.from_euler("xyz", com_rpy).as_matrix()
        for f in self.leg_names:
            assert f in feet_positions.keys()
            q_part = do_ik(feet_positions[f], R.dot(self.hip_positions[f][0]) + com_xyz[:3], self.hip_positions[f][1])
            if q_part is False:
                return False
            for k, key in enumerate(["hip_abduct_info", "hip_pitch_info", "knee_pitch_info"]):
                q[self.servo_configs[f][key]["pose_ind"]] = q_part[k]
        return q

    def publish_command(self, event):
        if not self.leg_interface.send_pose(self.joint_q, allow_projection=False):
            print("Current pose invalid.")

    def publish_state(self, event):
        state_msg = JointState()
        state_msg.header.seq = self.state_publish_seq
        self.state_publish_seq += 1
        state_msg.header.stamp = rospy.Time.from_sec(time.time())
        state_msg.header.frame_id = "world"
        
        state_msg.name = self.state_names
        state_msg.position = np.zeros(6+12)
        state_msg.velocity = np.zeros(6+12)
        state_msg.effort = np.zeros(6+12)
        state_msg.position[:6] = self.com_q[:]
        state_msg.position[6:] = self.joint_q[:]
        self.state_pub.publish(state_msg)

    def publish_status(self, event):
        status_msg = LocomotionServerStatus()
        status_msg.header.seq = self.status_publish_seq
        self.status_publish_seq += 1
        status_msg.header.stamp = rospy.Time.from_sec(time.time())
        status_msg.header.frame_id = ""
        status_msg.leg_names = self.leg_names
        status_msg.is_stance = [Bool(self.stance_status[k]) for k in self.leg_names]
        status_msg.feet_positions = [
            Vector3(*self.feet_positions[k]) for k in self.leg_names
        ]
        self.status_pub.publish(status_msg)

    @staticmethod    
    def is_stable_configuration(com_xyz, feet_positions, stance_status, contract_amount=0.0):
        # Create 3 x N matrices of the active and inactive (support / non-support) feet positions.
        active_feet = np.array([feet_positions[key] for key, value in stance_status.items() if value]).T.reshape(3, -1)
        inactive_feet = np.array([feet_positions[key] for key, value in stance_status.items() if not value]).T.reshape(3, -1)
        if active_feet.shape[1] <= 2:
            # 2 or less supporting feet will never be statically stable.
            return False

        if not in_hull(com_xyz[:2], active_feet[:2, :], contract_amount=contract_amount):
            return False

        # If it is in the hull, make sure all of the inactive feet don't interfere with the plane of support.
        # Easy hack, make sure it's above the highest support foot
        if inactive_feet.shape[1] > 0 and np.min(inactive_feet[2, :]) < np.max(active_feet[2, :]) - 1E-6:
            return False

        return True

    def handle_move_com(self, req):
        rep = MoveComResponse()
        
        rospy.loginfo("Req to move com to [%f, %f, %f, %f, %f, %f]" % (req.desired_xyz.x, req.desired_xyz.y, req.desired_xyz.z, req.desired_rpy.x, req.desired_rpy.y, req.desired_rpy.z))

        # Test if the desired COM is in the support polygon
        # of the current feet.
        desired_com_xyz = np.array([req.desired_xyz.x, req.desired_xyz.y, req.desired_xyz.z])
        desired_com_rpy = np.array([req.desired_rpy.x, req.desired_rpy.y, req.desired_rpy.z])
        feasible = self.is_stable_configuration(desired_com_xyz, self.feet_positions, self.stance_status)
        if not feasible:
            rospy.loginfo("\tFailed due to COM feasibility.")
            rep.success = Bool(False)
            rep.info = "Desired COM not stable w.r.t feet positions."
            return rep

        # It's feasible, so try to do IK for it.
        q = self.do_ik_for_feet_and_com(self.feet_positions, desired_com_xyz, desired_com_rpy)
        if q is False:
            rospy.loginfo("\tFailed due to IK feasibility.")
            rep.success = Bool(False)
            rep.info = "Desired COM failed IK: probably too far."
            return rep
        
        success = self.leg_interface.send_pose(q, allow_projection=False)
        if not success:
            rospy.loginfo("\tFailed due to servo constraints.")
            rep.success = Bool(False)
            rep.info = "Pose was not feasible w.r.t servo constraints."
            return rep
        
        # Commit changes to state.
        rospy.loginfo("\tSucceeded.")
        self.com_q[:3] = desired_com_xyz
        self.com_q[3:] = desired_com_rpy
        self.joint_q = q

        rep.success = Bool(True)
        rep.info = ""
        return rep

    def handle_move_foot(self, req):
        rep = MoveFootResponse()

        rospy.loginfo("Req to move foot %s to [%f, %f, %f] as support %s" % (req.leg_name, req.desired_xyz.x, req.desired_xyz.y, req.desired_xyz.z, req.end_in_support))

        # Make sure the foot choice is legit.
        if req.leg_name not in self.leg_names:
            rep.success = Bool(False)
            rep.info = "Invalid foot: " + req.leg_name
            return rep
        
        new_stance_status = deepcopy(self.stance_status)
        new_stance_status[req.leg_name] = False
        new_feet_positions = deepcopy(self.feet_positions)
        new_feet_positions[req.leg_name] = np.array([req.desired_xyz.x, req.desired_xyz.y, req.desired_xyz.z])
        if self.stance_status[req.leg_name] is True:
            # Make sure removing foot from support doesn't break COM, since
            # we might currently be relying on it.
            feasible = self.is_stable_configuration(
                self.com_q[:3],
                new_feet_positions,
                new_stance_status
            )
            if not feasible:
                rospy.loginfo("\tFailed due to COM feasibility.")
                rep.success = Bool(False)
                rep.info = "Removing foot " + req.leg_name + " would violate COM constraint."
                return rep
        
        # Figure out IK to move foot as desired.
        q = self.do_ik_for_feet_and_com(new_feet_positions, self.com_q)
        if q is False:
            rospy.loginfo("\tFailed due to IK feasibility.")
            rep.success = Bool(False)
            rep.info = "New foot position failed IK."
            return rep
        
        success = self.leg_interface.send_pose(q, allow_projection=False)

        if not success:
            rospy.loginfo("\tFailed due to servo constraints.")
            rep.success = Bool(False)
            rep.info = "Pose was not feasible w.r.t servo constraints."
            return rep
        
        if bool(req.end_in_support) is True:
            new_stance_status[req.leg_name] = True

        # Commit changes to state.
        rospy.loginfo("\tSucceeded.")
        self.joint_q = q
        self.feet_positions = new_feet_positions
        self.stance_status = new_stance_status

        rep.success = Bool(True )
        rep.info = ""
        return rep


def test():
    test_com = np.array([0., 0., 0.1])
    test_com_shifted = np.array([0.2, 0.2, 0.1])
    test_feet_stable_pos = {
        0: np.array([-1., -1., 0.]),
        1: np.array([-1., 1., 0.]),
        2: np.array([1., 1., 0.]),
        3: np.array([1., -1., 0.]),
    }
    all_support = {
        0: True,
        1: True,
        2: True,
        3: True
    }
    three_support = {
        0: False,
        1: True,
        2: True,
        3: True
    }
    two_support = {
        0: False,
        1: False,
        2: True,
        3: False
    }
    assert not LocomotionManager.is_stable_configuration(test_com, test_feet_stable_pos, two_support)
    assert not LocomotionManager.is_stable_configuration(test_com, test_feet_stable_pos, three_support) # marginally stable
    assert LocomotionManager.is_stable_configuration(test_com_shifted, test_feet_stable_pos, three_support) # marginally stable
    assert LocomotionManager.is_stable_configuration(test_com, test_feet_stable_pos, all_support)

def main():

    rospy.init_node('locomotion_server', anonymous=False)
            
    manager = LocomotionManager()
    
    t0 = time.time()
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Provide locomotion ROS actions.')
    parser.add_argument('--test', action="store_true")
    args = parser.parse_args()
    try:
        if args.test:
            test()
        else:
            main()
    except rospy.ROSInterruptException:
        pass




    
