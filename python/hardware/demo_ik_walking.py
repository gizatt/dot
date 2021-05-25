from leg_controller import *
from ik import single_leg_forward_kin
import rospy
import yaml
import time

'''
When run, does a handful of steps of forward walking.

For prototyping states in a very walking state machine.

Ordering of motions for a statically stable walk:

Given a walking vector:

1) Pick a foot to move.
2) Shift the COM away from that foot.
3) Move the foot in the walking vector direction.
4) Next foot to move is the next in a predetermined foot order.
   (Probably LF -> RH -> RF -> LH.)
'''

def do_ik(ee_target):
    angles = single_leg_forward_kin(
        ee_target=ee_target,
        hip_1_in_root_frame=np.zeros(3),
        hip_2_in_hip_1_frame=np.array([0.02, 0.02, 0.0]),
        hip_length=0.16,
        shin_length=0.16
    )
    return angles


def main():
    with open("servo_config.yaml") as f:
        servo_configs = yaml.load(f)

    rospy.init_node('test_ik_node', anonymous=False)

    q = np.zeros(12)
    leg_interface = HardwareInterface(q, servo_configs)

    t0 = time.time()
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():
        t = time.time() - t0
        # Flat circles, single leg
        #leg_angles = do_ik(np.array([0.1, 0.1, -0.2]) + np.array([np.cos(t)*0.1, np.sin(t)*0.1, np.sin(t)*0.00]))
        # Squats
        hz = 0.5
        leg_angles = do_ik(np.array([0.0, 0.025, -0.2]) + np.array([0., 0., np.sin(t*hz*2.*np.pi)*0.05]))
        if leg_angles is False:
            print("IK fail.")
            continue
        print("Leg angles ", leg_angles)
        for k in range(4):
            q[3*k : 3*k+3] = leg_angles[:]
        leg_interface.send_pose(q)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass




    