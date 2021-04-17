import maestro
import time
import numpy as np
import scipy as sp
import scipy.interpolate
import sys

# In degrees from straight config
pos_list = [
    np.array([0.0, 0.0, -45.]),
    np.array([0.0, 0.0, 45.]),
    np.array([0.0, 0.0, -45]),
    np.array([0.0, 90.0, -45]),
    np.array([0.0, 0.0, -45.]),
    np.array([-45.0, 00.0, -45]),
    np.array([0.0, 0.0, -45.]),
]
time_list = [0., 3.0, 6., 12., 15., 18, 21]

# Do interpolation
times = np.arange(min(time_list), max(time_list), 0.1)
pos_generator = sp.interpolate.interp1d(
    time_list, np.stack(pos_list, axis=0), axis=0,
    kind="linear",
    bounds_error=False,
    fill_value=(pos_list[0], pos_list[-1])
)

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
            pos = pos_generator(t)
            pos_in_us = servo_centers + pos * servo_ms_per_deg
            print("%f: %s -> %s" % (t, pos, pos_in_us))
            for k in range(n_servos):
                # Commands in quarter-ms
                servo.setTarget(k,int(pos_in_us[k]*4))
            servo.setTarget(5, int(pos_in_us[-2]*4))
            time.sleep(0.05)
    servo.close()