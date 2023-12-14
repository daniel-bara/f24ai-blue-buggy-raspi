# Author:   Thomas Foster
# Created:  29/03/22
# Purpose:  This file attempts to save the video output of the driver as a file to be watched later.
#           Is based off main_driver

# This script ...

import numpy as np
import cv2
import time
from menu import MainMenu
from camera.laptop_camera import LaptopCamera
from camera_perception import CVCameraPerceptionSlide
from path_planner import WallCentreLine

# Enter menu
menu = MainMenu()
menu.__init__()
sliders = menu.user_set()

# CREATE CLASS INSTANCES
# Laptop camera
cap = LaptopCamera.__init__()
saver = LaptopCamera.init_save_video()
# Cone recogniser
cv_perception = CVCameraPerceptionSlide(sliders) # not sure why I have to pass an input here??
cv_perception.__init__(sliders)
# Centre line creator
wall_line = WallCentreLine()
wall_line.__init__()


# Temp iterator
i = 0

# Init vehicle vectors
# vehicle_position, vehicle_forward, vehicle_right = vehicle_vectors
vehicle_vectors = [0,0,0]

# initialise lidar here

# allow the camera & lidar to warmup
time.sleep(0.1)

while True:
     
    image = LaptopCamera.get_frame(cap)

    [percept_frame, blue_p, yellow_p] = cv_perception.get_cones(image)

    [c_line, meta] = wall_line.path_generator(blue_p, yellow_p, vehicle_vectors)

    output = percept_frame
    for i in range(len(c_line)):
        if np.isnan(c_line[0][0]) == False:
            output = cv2.circle(output, (round(c_line[i][0]), round(c_line[i][1])), 3, [255, 255, 255], -1)

    # Show image
    cv2.imshow("Result", output)

    # Save image to file
    saver = LaptopCamera.save_frame(output, saver)

    key = cv2.waitKey(10) & 0xFF


    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

    # run at 10Hz
    cv2.waitKey(100)
  
# close window, release webcam
cap.release()
# Also release the saved video file
saver.release()
# De-allocate any associated memory usage 
cv2.destroyAllWindows()