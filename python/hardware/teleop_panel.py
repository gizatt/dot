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
from functools import partial
pn.extension()

import yaml
import rospy
from ros_utils import convert_np_vector_to_int16_multi_array
from std_msgs.msg import Int16MultiArray, String
from sensor_msgs.msg import JointState
from leg_controller import HardwareInterface

def do_onetime_setup():
    def exit_gracefully(signum, frame):
        sys.exit(1)
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)

    with open("servo_config.yaml") as f:
        servo_configs = yaml.load(f)
    q = np.zeros(12)
    pn.state.cache["leg_interface"] = HardwareInterface(q, servo_configs)

    class MicrosecondTeleopRow(pn.Row):
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
    class MicrosecondTeleopPanel(pn.Column):
        def __init__(self):
            self.servo_rows = [MicrosecondTeleopRow(i) for i in range(16)]
            super().__init__(
                *self.servo_rows
            )
        def get_us_cmd(self):
            us = np.zeros(16) - 1
            for row in self.servo_rows:
                if row.enable_checkbox.value:
                    us[row.i] = row.int_slider.value
            return us
        def get_unmasked_us(self):
            us = np.zeros(16) - 1
            for row in self.servo_rows:
                us[row.i] = row.int_slider.value
            return us
        def display_us(self, us):
            assert us.shape[0] == 16
            for i, row in enumerate(self.servo_rows):
                row.int_slider.value = int(us[i])
    pn.state.cache["microsecond_teleop_panel"] = MicrosecondTeleopPanel()
    
    class LegTeleopPanel(pn.Column):
        def __init__(self, leg, leg_name):
            self.sliders = []
            self.leg = leg
            for k, (info, name) in enumerate(zip(leg.infos, ["Hip abduct", "Hip pitch", "Knee pitch"])):
                self.sliders.append(
                    pn.widgets.FloatSlider(
                        name='%s' % name,
                        start=info.servo_min_rad, end=info.servo_max_rad,
                        value=0.,
                        step=0.01
                    )
                )
            super().__init__(
                *self.sliders,
                name=leg_name
            )
        def write_to_state(self, state_vec):
            assert state_vec.shape[0] == 12
            for info, slider in zip(self.leg.infos, self.sliders):
                state_vec[info.pose_ind] = slider.value
        def display_from_state(self, state_vec):
            assert state_vec.shape[0] == 12
            for info, slider in zip(self.leg.infos, self.sliders):
                slider.value = state_vec[info.pose_ind]

    class PresetPosesPanel(pn.GridBox):
        def __init__(self, parent):
            # Parent should be the PoseTeleopPanel
            assert isinstance(parent, PoseTeleopPanel)
            self.parent = parent
            with open("pose_library.yaml") as f:
                pose_library_dict = yaml.load(f)
            self.pose_library_buttons = []
            self.pose_library = {}
            sync_btn = pn.widgets.Button(name="Sync to State", button_type="success")
            sync_btn.on_click(self.on_sync_click)
            self.pose_library_buttons.append(sync_btn)
            for key in pose_library_dict.keys():
                pose = np.array([float(x) for x in pose_library_dict[key]])
                if pose.shape[0] != 12:
                    print("Ignoring key ", key, "from pose library dict: had shape ", pose.shape)
                    continue
                btn = pn.widgets.Button(name=key, button_type="primary")
                btn.on_click(self.on_click)
                self.pose_library_buttons.append(btn)
                self.pose_library[key] = pose
            super().__init__(
                *self.pose_library_buttons,
                name="Pose Library"
            )
        def on_click(self, event):
            self.parent.display_pose(self.pose_library[event.obj.name])
        def on_sync_click(self, event):
            self.parent.display_pose(pn.state.cache["last_joint_state"])

    class PoseTeleopPanel(pn.Accordion):
        def __init__(self):
            self.leg_panels = []
            self.interface = pn.state.cache["leg_interface"]
            for leg, leg_name in zip(self.interface.legs, self.interface.leg_names):
                self.leg_panels.append(LegTeleopPanel(leg, leg_name))
            self.preset_poses_panel = PresetPosesPanel(self)
            super().__init__(
                self.preset_poses_panel,
                *self.leg_panels,
                active=list(range(len(self.leg_panels)))
            )
        def get_state_cmd(self):
            q = np.zeros(12)
            for panel in self.leg_panels:
                panel.write_to_state(q)
            return q
        def display_pose(self, q):
            assert q.shape[0] == 12
            for panel, leg in zip(self.leg_panels, self.interface.legs):
                panel.display_from_state(q)

    pn.state.cache["pose_teleop_panel"] = PoseTeleopPanel()

    class MicrosecondStatePanel(pn.Row):
        def __init__(self):
            self.servo_indicators = [
                pn.indicators.Number(
                    name='%02d' % k, value=-1, format='{value}us',
                    colors=[(0, 'red'), (1, 'green')],
                    font_size="18", title_size="16"
                )
                for k in range(16)
            ]
            super().__init__(
                *self.servo_indicators
            )
        def try_set_state(self, data):
            try:
                xs = [int(x) for x in data.strip().split(" ")]
                if len(xs) == 16:
                    for k in range(16):
                        self.servo_indicators[k].value = xs[k]
            except ValueError:
                pass
    pn.state.cache["microsecond_state_panel"] = MicrosecondStatePanel()

    pn.state.cache["message_box"] = pn.widgets.StaticText(name='Latest Chatter:', value='<none>')
    pn.state.cache["command_select_panel"] = pn.widgets.RadioButtonGroup(
        name='Command Select', options=['None', 'Pose', 'Microsecond'], button_type='success'
    )

    teleop_panel = pn.Row(
        pn.state.cache["pose_teleop_panel"], pn.Spacer(width=200), pn.state.cache["microsecond_teleop_panel"]
    )
    # Make the interface for serving
    pn.state.cache["page"] = pn.Column(
        pn.state.cache["message_box"],
        pn.state.cache["microsecond_state_panel"],
        pn.state.cache["command_select_panel"],
        teleop_panel
    )

def chatter_callback(data):
    pn.state.cache["message_box"].value = data.data
    pn.state.cache["microsecond_state_panel"].try_set_state(data.data)
def robot_state_callback(data):
    pn.state.cache["last_joint_state"]= np.array(data.position[6:])

def main_loop():
    t0 = time.time()
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():

        # Core logic: command based on which panel is active.
        active_panel = pn.state.cache["command_select_panel"].value
        if active_panel == "Microsecond":
            us = pn.state.cache["microsecond_teleop_panel"].get_us_cmd()
            #print("Sending us: ", str(["%04d" % u for u in us]))
            pn.state.cache["leg_interface"].send_us(us)
            # Also update the pose panel
            unmasked_us = pn.state.cache["microsecond_teleop_panel"].get_unmasked_us()
            pn.state.cache["pose_teleop_panel"].display_pose(
                pn.state.cache["leg_interface"].convert_us_to_pose(unmasked_us)
            )

        elif active_panel == "Pose":
            q = pn.state.cache["pose_teleop_panel"].get_state_cmd()
            #print("Sending pose: ", str(["%0.02f" % x for x in q]))
            pn.state.cache["leg_interface"].send_pose(q)
            # Also update the us panel
            pn.state.cache["microsecond_teleop_panel"].display_us(
                pn.state.cache["leg_interface"].curr_us
            )

        rate.sleep()

if "onetime_setup" not in pn.state.cache.keys():
    do_onetime_setup()
    pn.state.cache["last_joint_state"] = np.zeros(12)
    pn.state.cache["onetime_setup"] = True

pn.state.cache["page"].servable()

if "thread" not in pn.state.cache.keys():
    rospy.init_node('direct_teleop_controller', anonymous=False)
    rospy.Subscriber("chatter", String, chatter_callback)
    rospy.Subscriber("/robot_state", JointState, robot_state_callback)

    pn.state.cache["thread"] = threading.Thread(target=main_loop)
    pn.state.cache["thread"].start()