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
from ros_utils import convert_np_vector_to_int16_multi_array
from std_msgs.msg import Int16MultiArray

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
    def __init__(self, q0, leg_config):
        # Leg config is a subset of the YAML config for the robot.
        self.hip_abduct_info = ServoInfo(leg_config["hip_abduct_info"])
        self.hip_pitch_info = ServoInfo(leg_config["hip_pitch_info"])
        self.knee_pitch_info = ServoInfo(leg_config["knee_pitch_info"])
        # Infos in their canonical order that matches q.
        self.infos = [self.hip_abduct_info, self.hip_pitch_info, self.knee_pitch_info]
        self.fourbar_eps = 30. * np.pi/180. # 45 deg safety for fourbar
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
        return us

class HardwareInterface():
    '''
        Generates servo microsecond commands for the entire leg.
        Takes simple joint limits into account (on a per-leg level),
        but doesn't prevent all bad configs, so be wary.
    '''
    def __init__(self, q0, config):
        # q0 is a 16-length np vector.
        assert(q0.shape == (16,))
        # Config is a YAML config containing the individual leg configs.
        self.left_front_leg = LegHardwareInterface(
            q0[0:3],
            config["left_front_leg"]
        )

        self.pub = rospy.Publisher('motor_command', Int16MultiArray, queue_size=1)

    def convert_pose_to_us(self, q):
        raise NotImplementedError("Pose to servo ind mapping")
        assert q.shape == (16,)
        us = np.zeros(16) - 1
        us[0:3] = self.left_front_leg.convert_pose_to_us(q[:3])
        return us
    
    def send_pose(self, q):
        assert q.shape == (16,)
        self.pub.publish(
            convert_np_vector_to_int16_multi_array(
                self.convert_pose_to_us(q)
        ))
    def send_us(self, us):
        ''' WARNING: NO SANITY CHECKING! '''
        assert us.shape == (16,)
        self.pub.publish(
            convert_np_vector_to_int16_multi_array(us)
        )

def main():
    with open("servo_config.yaml") as f:
        servo_configs = yaml.load(f)

    rospy.init_node('leg_controller', anonymous=False)

    q = np.zeros(16)
    leg_interface = HardwareInterface(q, servo_configs)

    t0 = time.time()
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():
        q[2] = np.sin(time.time() - t0) * 0
        leg_interface.send_pose(q)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass






