'''
Panel app to apply direct pose commands to the robot.

Servo with:
`panel serve direct_teleop_panel.py --allow-websocket-origin=192.168.0.142:5006`

(add "--dev" for hot reloading for dev work).

Panel layout:

  "Enable"     "Value"
  <Checkbox> <Int slider>
'''

import math
import numpy as np
import time
import threading
import signal
import panel as pn
import sys
pn.extension()

import yaml
import rospy
from ros_utils import convert_np_vector_to_int16_multi_array
from std_msgs.msg import Int16MultiArray
from leg_controller import HardwareInterface

class ServoRow(pn.Row):
    def __init__(self, i):
        self.i = i
        self.int_slider = pn.widgets.IntSlider(
            name='Servo Control %02d' % i,
            start=600, end=2500,
            value=1500,
            step=1
        )
        self.enable_checkbox = pn.widgets.Checkbox(
            name='Toggle %02d' % i
        )
        super().__init__(
            self.enable_checkbox, self.int_slider
        )


# Make the interface for serving
rows = [ServoRow(i) for i in range(16)]
page = pn.Column(*rows)
page.servable()

def get_us_from_rows():
    us = np.zeros(16) - 1
    for row in rows:
        if row.enable_checkbox.value:
            us[row.i] = row.int_slider.value
    return us



def main():
    with open("servo_config.yaml") as f:
        servo_configs = yaml.load(f)
    q = np.zeros(16)
    leg_interface = HardwareInterface(q, servo_configs)

    t0 = time.time()
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():
        us = get_us_from_rows()
        print("Sending ", str(["%04d" % u for u in us]))
        leg_interface.send_us(us)
        rate.sleep()

def exit_gracefully(signum, frame):
    sys.exit(1)
original_sigint = signal.getsignal(signal.SIGINT)
signal.signal(signal.SIGINT, exit_gracefully)

rospy.init_node('direct_teleop_controller', anonymous=False)
thread = threading.Thread(target=main)
thread.start()