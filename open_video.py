# Show video display only

import time
from time import time as tn
import os
import numpy as np
from numpy import pi
import cv2
import sys
from camera.pi_camera import RPiCamera


# VERTICAL_RES = 360

# RES = [int(VERTICAL_RES*4/3),VERTICAL_RES]

RES=[480,368]

# Create class instance, get frame to warm up
cap = RPiCamera(RES,0)

while True:

    image = cap.get_frame()
    image = cv2.flip(image,1)

    cv2.imshow("Live Feed", image)

    key = cv2.waitKey(10) & 0xFF

    if key == ord("q"):
        
        break
