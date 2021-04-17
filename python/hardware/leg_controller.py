from copy import deepcopy
import maestro
import time
import numpy as np
import scipy as sp
import scipy.interpolate
import sys
from collections import namedtuple
import logging

class LegHardwareInterface():
    '''
        Commands a given leg using the supplied maestro controller.
        Takes some simple joint limits into account to keep the
        leg from being driven into bad configurations.
        (Does not prevent all bad configs, so be careful!
    '''
    # ServoInfo is descriptive enough to tell how to put a given
    # servo at a particular angle, where 0* is hip abduct straight out,
    # hip straight down, knee straight forward.

    class ServoInfo:
        def __init__(self, maestro, ind, servo_0deg_us, servo_90deg_us, servo_min_us, servo_max_us):
            self.maestro = maestro
            self.ind = ind
            self.servo_0deg_us = servo_0deg_us
            self.servo_90deg_us = servo_90deg_us
            self.servo_min_us = servo_min_us
            self.servo_max_us = servo_max_us
            self.servo_us_per_rad = (servo_90deg_us - servo_0deg_us) / (np.pi/2.)
            # Handle whether angles and us coordinates are flipped or not.
            servo_bound_1 = self.convert_us_to_rad(servo_min_us)
            servo_bound_2 = self.convert_us_to_rad(servo_max_us)
            self.servo_min_rad = min(servo_bound_1, servo_bound_2)
            self.servo_max_rad = max(servo_bound_1, servo_bound_2)

            print("Servo %d bounds " % ind, self.servo_min_rad, self.servo_max_rad)
        def convert_rad_to_us(self, rad):
            return rad * self.servo_us_per_rad + self.servo_0deg_us
        def convert_us_to_rad(self, us):
            return (us - self.servo_0deg_us) / self.servo_us_per_rad



    def __init__(self, q0, hip_abduct_info, hip_pitch_info, knee_pitch_info):
        self.hip_abduct_info = hip_abduct_info
        self.hip_pitch_info = hip_pitch_info
        self.knee_pitch_info = knee_pitch_info
        self.fourbar_eps = 30. * np.pi/180. # 45 deg safety for fourbar
        self.curr_pose = None
        self.set_leg_posture(q0)

    def convert_pose_command_to_feasible_pose(self, q):
        # Given a pose target in radians, converts to a feasible
        # (w.r.t fourbar and servo limit constraints) set of microsecond
        # commands to apply.
        bounded_q = deepcopy(q)
        knee_lb = q[1] - np.pi/2. + self.fourbar_eps
        knee_ub = q[1] + np.pi/2. - self.fourbar_eps
        bounded_q[2] = np.clip(bounded_q[2], knee_lb, knee_ub)
        
        for k, info in enumerate([self.hip_abduct_info, self.hip_pitch_info, self.knee_pitch_info]):
            bounded_q[k] = np.clip(bounded_q[k], info.servo_min_rad, info.servo_max_rad)
        
        if not np.allclose(bounded_q, q):
            logging.warning("Bounding q from %s to %s.", q, bounded_q)

        return bounded_q

    def set_leg_posture(self, q):
        # Ordering hip_abduct, hip_pitch, knee_pitch
        # Enforce their individual bound limits, but also
        # constraint that the knee angle must be within -90 + eps,
        # and 90 - eps of the hip angle to not break the four bar.

        bounded_q = self.convert_pose_command_to_feasible_pose(q)
        for k, info in enumerate([self.hip_abduct_info, self.hip_pitch_info, self.knee_pitch_info]):
            pos_in_us = info.convert_rad_to_us(bounded_q[k])

            # Commit command
            info.maestro.setTarget(info.ind,int(pos_in_us*4))
        self.curr_pose = bounded_q

    def slew_to_posture(self, qtarg, qd=np.pi/2., dt=0.01):
        # qd in deg/sec
        qtarg = self.convert_pose_command_to_feasible_pose(qtarg)
        max_step_allowed = qd * dt
        while (1):
            qerr = qtarg - self.curr_pose
            if np.sum(np.abs(qerr)) < 1e-6:
                return
            step_applied = np.clip(qerr, -max_step_allowed, max_step_allowed)
            self.set_leg_posture(self.curr_pose + step_applied)
            last_applied_time = time.time()
            while time.time() - last_applied_time < dt:
                time.sleep(time.time() - last_applied_time)
        

if __name__ == "__main__":
    servo = maestro.Controller(ttyStr='/dev/ttyACM0')

    hip_abduct_info = LegHardwareInterface.ServoInfo(
        maestro=servo,
        ind=0,
        servo_0deg_us=1480,
        servo_90deg_us=840,
        servo_min_us=1200, # 800 is straight up, but too big for my test mount
        servo_max_us=1450 # more reaches under the robot but hits test stand
    )
    hip_pitch_info = LegHardwareInterface.ServoInfo(
        maestro=servo,
        ind=1,
        servo_0deg_us=1480,
        servo_90deg_us=2150,
        servo_min_us=1200,
        servo_max_us=2350
    )
    knee_pitch_info = LegHardwareInterface.ServoInfo(
        maestro=servo,
        ind=2,
        servo_0deg_us=1480,
        servo_90deg_us=815,
        servo_min_us=550,
        servo_max_us=2100
    )
    
    
    leg_hardware_interface = LegHardwareInterface(
        np.zeros(3),
        hip_abduct_info,
        hip_pitch_info,
        knee_pitch_info
    )

    infos = [hip_abduct_info, hip_pitch_info, knee_pitch_info]
    lbs = np.array([info.servo_min_rad for info in infos])
    ubs = np.array([info.servo_max_rad for info in infos])
    rng = np.random.default_rng()
    for k in range(10):
        qtarg = rng.uniform(low=lbs, high=ubs)
        leg_hardware_interface.slew_to_posture(qtarg)
        time.sleep(1.)
    leg_hardware_interface.slew_to_posture(np.array([0, 0, 0]))
    servo.close()
