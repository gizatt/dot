#!/usr/bin/env python
import rospy
import numpy as np
import time
import random
import sys
from dot_msgs.msg import ServoSetTrajectory

def talker():
    pub = rospy.Publisher('due_joint_trajectory_cmd', ServoSetTrajectory, queue_size=1)
    rospy.init_node('test_trajectory_pub', anonymous=True)

    test_trajectory_msg = ServoSetTrajectory()
    test_trajectory_msg.header.stamp = rospy.Time.now()
    
    nq = 16
    nb = 5
    test_trajectory_msg.num_positions = nq
    test_trajectory_msg.num_breaks = nb

    data = np.zeros((nb, nq), dtype=np.int16) - 1

    data[0, 5] = 1100
    data[1, 5] = 1700
    data[2, 5] = 1100
    data[3, 5] = 1700
    data[4, 5] = 1100
        
    test_trajectory_msg.data = data.flatten().tolist()
    test_trajectory_msg.breaks_from_start = [1., 2., 2.5, 2.8, 3.]
    pub.publish(test_trajectory_msg)

if __name__ == '__main__':
    try:
        for k in range(1):
            talker()
            time.sleep(1.)
    except rospy.ROSInterruptException:
        pass
