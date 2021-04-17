import os
import time
import sys
import yaml

import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

import maestro
from apriltag import apriltag
from realsense_handler import RealsenseHandler
from leg_controller import LegHardwareInterface

# Setup realsense and tag detector
realsense_manager = RealsenseHandler()
detector = apriltag(family="tag16h5", debug=True, threads=8)
cv2.namedWindow("detections")

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

rng = np.random.default_rng()

# Main loop: command to posture, wait till it gets there,
# do a detection, and save output.

def do_apriltag_detection_and_save():
    color_image, _, _ = realsense_manager.get_frame(include_pointcloud=False)
    color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray_image)
    print("REALSENSE DETECTIONS:", detections)
    if len(detections) == 3:
        print("Correct number, saving detections.")
        # Fix numpy arrays to lists for more human-readable storage
        for detection in detections:
            detection["center"] = detection["center"].tolist()
            detection["lb-rb-rt-lt"] = detection["lb-rb-rt-lt"].tolist()
        out = [{
            "commanded_pose": leg_hardware_interface.curr_pose.tolist(),
            "detections": list(detections)
        }]
        with open("calibrations.yaml", "a") as f:
            yaml.dump(out, f)
    detection_image = cv2.imread("debug_output.pnm")
    cv2.imshow("detections", detection_image)
    cv2.waitKey(1)


try:
    while (1):
        qtarg = rng.uniform(low=leg_hardware_interface.q_lb, high=leg_hardware_interface.q_ub)
        qactual = leg_hardware_interface.slew_to_posture(qtarg, qd=np.pi)
        time.sleep(1.)
        do_apriltag_detection_and_save()
except Exception as e:
    print("Exception ", e)
except KeyboardInterrupt:
    print("Cleaning up")
leg_hardware_interface.slew_to_posture(np.array([0, 0, 0]))
servo.close()