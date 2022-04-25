from copy import deepcopy
import time
import numpy as np
import scipy as sp
import scipy.interpolate
import sys
from collections import namedtuple
import logging
import yaml

import rospy
from dot_msgs.msg import ServoSetTrajectory


class ServoInfo:
    # ServoInfo is descriptive enough to tell how to put a given
    # servo at a particular angle, where 0* is hip abduct straight out,
    # hip straight down, knee straight forward.
    def __init__(self, servo_config):
        # Servo config is a subset of the YAML config for the
        # leg.
        self.servo_ind = servo_config["servo_ind"]
        self.pose_ind = servo_config["pose_ind"]
        self.servo_0deg_us = servo_config["servo_0deg_us"]
        self.servo_90deg_us = servo_config["servo_90deg_us"]
        self.servo_min_us = servo_config["servo_min_us"]
        self.servo_max_us = servo_config["servo_max_us"]
        self.servo_us_per_rad = (self.servo_90deg_us - self.servo_0deg_us) / (np.pi/2.)
        # Handle whether angles and us coordinates are flipped or not.
        servo_bound_1 = self.convert_us_to_rad(self.servo_min_us)
        servo_bound_2 = self.convert_us_to_rad(self.servo_max_us)
        self.servo_min_rad = min(servo_bound_1, servo_bound_2)
        self.servo_max_rad = max(servo_bound_1, servo_bound_2)
    def convert_rad_to_us(self, rad):
        return rad * self.servo_us_per_rad + self.servo_0deg_us
    def convert_us_to_rad(self, us):
        return (us - self.servo_0deg_us) / self.servo_us_per_rad
        
class LegHardwareInterface():
    '''
        Generates servo microsecond commands for a given leg.
        Takes some simple joint limits into account to keep the
        leg from being driven into bad configurations.
        (Does not prevent all bad configs, so be careful!)
    '''
    def __init__(self, leg_config):
        # Leg config is a subset of the YAML config for the robot.
        self.hip_abduct_info = ServoInfo(leg_config["hip_abduct_info"])
        self.hip_pitch_info = ServoInfo(leg_config["hip_pitch_info"])
        self.knee_pitch_info = ServoInfo(leg_config["knee_pitch_info"])
        # Infos in their canonical order that matches q.
        self.infos = [self.hip_abduct_info, self.hip_pitch_info, self.knee_pitch_info]
        self.fourbar_eps = 45. * np.pi/180. # 45 deg safety for fourbar
        self.q_lb = np.array([info.servo_min_rad for info in self.infos])
        self.q_ub = np.array([info.servo_max_rad for info in self.infos])

    def convert_pose_command_to_feasible_pose(self, q):
        # Given a pose target in radians, converts to a feasible
        # (w.r.t fourbar and servo limit constraints) set of microsecond
        # commands to apply.
        bounded_q = deepcopy(q)
        knee_lb = q[1] - np.pi/2. + self.fourbar_eps
        knee_ub = q[1] + np.pi/2. - self.fourbar_eps
        new_knee = np.clip(bounded_q[2], knee_lb, knee_ub)
        bounded_q[2] = new_knee
        
        bounded_q = np.clip(bounded_q, self.q_lb, self.q_ub)
        if not np.allclose(bounded_q, q):
            logging.warning("Bounding q from %s to %s.", q, bounded_q)

        return bounded_q

    def convert_pose_to_us(self, q):
        # Ordering hip_abduct, hip_pitch, knee_pitch
        # Enforce their individual bound limits, but also
        # constraint that the knee angle must be within -90 + eps,
        # and 90 - eps of the hip angle to not break the four bar.

        bounded_q = self.convert_pose_command_to_feasible_pose(q)
        us = np.array([info.convert_rad_to_us(bounded_q[k]) for k, info in enumerate(self.infos)])
        return us, not np.allclose(bounded_q, q)

    def convert_us_to_pose(self, us):
        # Ordering hip_abduct, hip_pitch, knee_pitch
        q = np.array([info.convert_us_to_rad(us[k]) for k, info in enumerate(self.infos)])
        return q

class HardwareInterface():
    '''
        Generates servo microsecond commands for the entire leg.
        Takes simple joint limits into account (on a per-leg level),
        but doesn't prevent all bad configs, so be wary.
    '''
    def __init__(self, q0, config):
        # q0 is a 12-length np vector.
        assert(q0.shape == (12,))
        # Config is a YAML config containing the individual leg configs.
        self.leg_names = config.keys()
        self.us_index_per_q = np.zeros(12).astype(np.int)
        self.q_inds_per_leg = []
        self.us_inds_per_leg = []
        self.legs = []
        for leg_name in self.leg_names:
            leg = LegHardwareInterface(config[leg_name])
            self.legs.append(leg)
            # Built mapping from q to us.
            q_inds_for_leg = []
            us_inds_for_leg = []
            for info in leg.infos:
                q_inds_for_leg.append(info.pose_ind)
                us_inds_for_leg.append(info.servo_ind)
            self.q_inds_per_leg.append(q_inds_for_leg)
            self.us_inds_per_leg.append(us_inds_for_leg)

        self.pub = rospy.Publisher('speck/joint_trajectory_cmd', ServoSetTrajectory, queue_size=1)

        self.curr_pose = q0
        self.curr_us, _ = self.convert_pose_to_us(q0)

    def convert_pose_to_us(self, q):
        assert q.shape == (12,)
        us = np.zeros(16) - 1
        required_projection = False
        for leg, q_inds, us_inds, leg_name in zip(self.legs, self.q_inds_per_leg, self.us_inds_per_leg, self.leg_names):
            us[us_inds], required_projection_this = leg.convert_pose_to_us(q[q_inds])
            if required_projection_this:
                required_projection = required_projection_this
        return us, required_projection
    
    def convert_us_to_pose(self, us):
        assert us.shape == (16,)
        q = np.zeros(12)
        for leg, q_inds, us_inds in zip(self.legs, self.q_inds_per_leg, self.us_inds_per_leg):
            q[q_inds] = leg.convert_us_to_pose(us[us_inds])
        return q

    def send_pose(self, q, allow_projection=True):
        assert q.shape == (12,)
        us_candidate, required_projection = self.convert_pose_to_us(q)
        if allow_projection is False and required_projection is True:
            return False
        self.send_us(us_candidate)
        return True

    def send_us(self, us):
        ''' WARNING: NO SANITY CHECKING! '''
        assert us.shape == (16,)
        self.curr_us = us
        self.curr_pose = self.convert_us_to_pose(us)
    
        trajectory_msg = ServoSetTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        nq = 16
        nb = 1
        trajectory_msg.num_positions = nq
        trajectory_msg.num_breaks = nb
        data = np.zeros((nb, nq), dtype=np.int16) - 1
        data[0, :] = us
        trajectory_msg.breaks_from_start = [0.]
        trajectory_msg.data = data.flatten().tolist()    
        self.pub.publish(trajectory_msg)

def main():
    with open("servo_config.yaml") as f:
        servo_configs = yaml.load(f)

    rospy.init_node('leg_controller', anonymous=False)

    q = np.zeros(12)
    leg_interface = HardwareInterface(q, servo_configs)

    t0 = time.time()
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():
        leg_interface.send_pose(q)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass






