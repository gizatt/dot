#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

def talker(i, v):
    pub = rospy.Publisher('motor_command', Int16MultiArray, queue_size=1)
    rospy.init_node('motor_command_publisher', anonymous=True)

    cmd_array = [-1] * 16
    cmd_array[i] = v
    servo_cmd = Int16MultiArray(
        layout=MultiArrayLayout(
            dim=[MultiArrayDimension(
                label="",
                size=16,
                stride=1
            )],
            data_offset=0
        ),
        data=cmd_array)
    pub.publish(servo_cmd)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("servo_num", type=int)
    parser.add_argument("servo_cmd", type=int)
    args = parser.parse_args()
    try:
        talker(args.servo_num, args.servo_cmd)
    except rospy.ROSInterruptException:
        pass