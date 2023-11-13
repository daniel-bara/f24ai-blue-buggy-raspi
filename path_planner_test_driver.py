# This script is used to test the functionality of the path planner algorithm
# Originally copied from main_driver

import numpy as np
import cv2
import time
from camera.laptop_camera import LaptopCamera
from camera_perception import CVCameraPerception
from path_planner import WallCentreLine

# Create class instances
cap = LaptopCamera.__init__()

cv_perception = CVCameraPerception()
cv_perception.__init__()

wall_line = WallCentreLine()
wall_line.__init__()

# Init vehicle vectors
# vehicle_position, vehicle_forward, vehicle_right = vehicle_vectors
# vehicle vectors set to the approximate position of the front axis of vehicle relative to camera origin
vehicle_vectors = [0,550,320]

# initialise lidar here

# allow the camera & lidar to warmup
time.sleep(0.1)

image = LaptopCamera.get_frame(cap)

# Create arbitrary cone locations
blue_p = [(175, 25, 200), (125, 225, 200), (75, 360, 200), (25, 470, 200)]
yellow_p = [(475, 50, 200), (525, 225, 200), (575, 360, 200), (625, 470, 200)]

[c_line, meta] = wall_line.path_generator(blue_p, yellow_p, vehicle_vectors)

for i in range(len(blue_p)):
    image = cv2.circle(image, (blue_p[i][0], blue_p[i][1]), 3, [255, 0, 0], -1)
    

for i in range(len(yellow_p)):
    image = cv2.circle(image, (yellow_p[i][0], yellow_p[i][1]), 3, [0, 255, 255], -1)

print('Centre line : ', c_line)

print('length = ', len(c_line))

print(c_line[0][0])
print(c_line[0][1])

for i in range(len(c_line)):
    print(i)
    image = cv2.circle(image, (round(c_line[i][0]), round(c_line[i][1])), 3, [0, 0, 255], -1)

cv2.imshow("Result", image)

while True:
    key = cv2.waitKey(10) & 0xFF


    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

    cv2.waitKey(10)
