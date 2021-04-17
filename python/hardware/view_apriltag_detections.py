import glob
import os
import random
import time
import sys
import asyncio 

import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyglet
import pyglet.gl as gl
import OpenGL.GL as gl_better

from apriltag import apriltag

from realsense_handler import RealsenseHandler

realsense_manager = RealsenseHandler()
detector = apriltag(family="tag16h5", debug=True, threads=8)
cv2.namedWindow("detections")

while (1):
    color_image, _, _ = realsense_manager.get_frame(include_pointcloud=False)
    color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray_image)
    print("REALSENSE DETECTIONS:", detections)
    detection_image = cv2.imread("debug_output.pnm")
    cv2.imshow("detections", detection_image)
    cv2.waitKey(1)
