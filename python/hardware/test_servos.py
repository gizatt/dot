import maestro
import time
import numpy as np
import scipy as sp
import scipy.interpolate
import sys

a1 = 0.1
a2 = 0.26
# In degrees from straight config
ee_pos_list = [
    np.array([0.12, 0.17]),
    #np.array([0.12, 0]),
    #np.array([0.14, 0]),
    np.array([0.325, 0.12]),
    np.array([0.12, 0.17]),
]
time_list = [0., 1., 2.]


# Automatically cycle smoothly

# Do interpolation
times = np.arange(min(time_list), max(time_list), 0.1)
ee_pos_generator = sp.interpolate.interp1d(
    time_list, np.stack(ee_pos_list, axis=0), axis=0,
    kind="linear",
    bounds_error=False,
    fill_value=(ee_pos_list[0], ee_pos_list[-1])
)

def do_ik(pos):
    # https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
    q2 = np.arccos( (pos[0]**2 + pos[1]**2 - a1**2 - a2**2) / (2 * a1 * a2) )
    q1 = np.arctan2(pos[1], pos[0]) - np.arctan2((a2 * np.sin(q2)), (a1 + a2 * np.cos(q2)))
    # 0 for q2 is actually 90*
    return np.array([0, q1, q2 - np.pi/2.])*180./np.pi

n_servos = 3
servo_min = np.array([500, 500, 500])
servo_centers = np.array([1500, 1500,  1500])
servo_ms_per_deg = np.array([1000/90., 1000/90., 1000/90.])

if __name__ == "__main__":
    servo = maestro.Controller(ttyStr='COM6')

    while (1):
        start_time = time.time()
        t = time.time() - start_time
        while t < max(time_list):
            t = time.time() - start_time
            ee_pos = ee_pos_generator(t)
            pos = do_ik(ee_pos)
            print("%f: %s -> %s" % (t, ee_pos, pos))
            pos_in_ms = servo_centers + pos * servo_ms_per_deg
            for k in range(n_servos):
                # Commands in quarter-ms
                servo.setTarget(k,int(pos_in_ms[k]*4))
    servo.close()