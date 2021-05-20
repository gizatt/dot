'''
Panel app to apply direct pose commands to the robot.

Servo with:
`panel serve direct_teleop_panel.py --allow-websocket-origin=192.168.0.142:5006`

(add "--dev" for hot reloading for dev work).

Hackily uses pn.state.cache to store the ROS node info and core interaction
state; each view on a page gets a thread that updates the state from
the cache. Not very intelligent / takes a lot of ugly code; this probably
isn't the right framework for this job...
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
from std_msgs.msg import Int16MultiArray, String
from leg_controller import HardwareInterface

def do_onetime_setup():
    def exit_gracefully(signum, frame):
        sys.exit(1)
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)

    with open("servo_config.yaml") as f:
        servo_configs = yaml.load(f)
    q = np.zeros(16)
    pn.state.cache["leg_interface"] = HardwareInterface(q, servo_configs)

    class ServoRow(pn.Row):
        def __init__(self, i):
            self.i = i
            self.int_slider = pn.widgets.IntSlider(
                name='Servo Control %02d' % i,
                start=200, end=3000,
                value=1500,
                step=1
            )
            self.enable_checkbox = pn.widgets.Checkbox(
                name='Toggle %02d' % i
            )
            super().__init__(
                self.enable_checkbox, self.int_slider
            )

    pn.state.cache["message_box"] = pn.widgets.StaticText(name='Latest Chatter:', value='<none>')
    pn.state.cache["servo_indicators"] = [
        pn.indicators.Number(
            name='%02d' % k, value=-1, format='{value}us',
            colors=[(0, 'red'), (1, 'green')],
            font_size="18", title_size="16"
        )
        for k in range(16)
    ]
    pn.state.cache["servo_rows"] = [ServoRow(i) for i in range(16)]

def get_us_from_rows():
    us = np.zeros(16) - 1
    for row in pn.state.cache["servo_rows"]:
        if row.enable_checkbox.value:
            us[row.i] = row.int_slider.value
    return us

def chatter_callback(data):
    pn.state.cache["message_box"].value = data.data
    try:
        xs = [int(x) for x in data.data.strip().split(" ")]
        if len(xs) == 16:
            for k in range(16):
                pn.state.cache["servo_indicators"][k].value = xs[k]
    except ValueError:
        pass

def main_loop():
    t0 = time.time()
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():
        us = get_us_from_rows()
        print("Sending ", str(["%04d" % u for u in us]))
        pn.state.cache["leg_interface"].send_us(us)
        rate.sleep()

if "onetime_setup" not in pn.state.cache.keys():
    do_onetime_setup()
    pn.state.cache["onetime_setup"] = True

# Make instance of page from shared viewables.
indicator_row = pn.Row(*pn.state.cache["servo_indicators"])
# Make the interface for serving
page = pn.Column(pn.state.cache["message_box"], indicator_row, *pn.state.cache["servo_rows"])
page.servable()

if "thread" not in pn.state.cache.keys():
    rospy.init_node('direct_teleop_controller', anonymous=False)
    rospy.Subscriber("chatter", String, chatter_callback)

    pn.state.cache["thread"] = threading.Thread(target=main_loop)
    pn.state.cache["thread"].start()