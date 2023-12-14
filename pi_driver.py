# This script ...

import os
import numpy as np
import cv2
import time
from subprocess import call as spc
from menu import MainMenu
from HexConv import steerCAN, motorCAN
from camera.pi_camera import RPiCamera
from camera_perception import CVCameraPerception
from path_planner import WallCentreLine
# from path_follower.path_follower_update import PathFollowing
from path_follower.arc_path_follower import PathFollowing
import RPi.GPIO as GPIO
    
def Stats(t,i): # Print timing stats following stop command

    print('\n')
    print('Total runtime:         ' + str(round(t,3)) + ' seconds' )
    print('Total frames:          ' + str(i) + ' frames' )
    print('Average frame rate:    ' + str(round(i/t,2)) + ' Hz\n')
    
GPIO.setmode(GPIO.BCM)
GPIO.setup(21,GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
GPIO.add_event_detect(21,GPIO.FALLING,bouncetime=2000)

# Enter menu
menu = MainMenu()
menu.__init__()
sliders = menu.user_set()

# Set up CAN interface
bashCommand = 'sudo ip link set can0 up type can bitrate 500000'
spc(bashCommand, shell=True)

# Create class instances
cap, raw = RPiCamera.__init__()
saver = RPiCamera.init_save_video()

cv_perception = CVCameraPerception(sliders) # not sure why I have to pass an input here??
cv_perception.__init__(sliders)

wall_line = WallCentreLine()
wall_line.__init__()

path_follower = PathFollowing()
path_follower.__init__()

# Temp iterator
i = 0

# Init vehicle vectors
# vehicle_position, vehicle_forward, vehicle_right = vehicle_vectors
vehicle_vectors = [0,0,0]

# initialise lidar here
########

# allow the camera & lidar to warmup, and time to unplug eth
time.sleep(5)

# Motor ramp vars
pwr_MAX = 50
t_RAMP = 2.5

SEND_COMMS = 0

W = 0.5
pwr = 0
tT = 0
n_frames = 0

for frame in cap.capture_continuous(raw, format="bgr", use_video_port=True):
    
    print('Frame ' + str(int(n_frames)))

    t0 = time.time()

    raw.truncate(0)
    image = frame.array

    [percept_frame, blue_p, yellow_p] = cv_perception.get_cones(image)
    output = percept_frame
    
    [c_line, meta] = wall_line.path_generator(blue_p, yellow_p, vehicle_vectors)
    
    angle = path_follower.get_steer(c_line, 1, 50, image)

    if angle is None:
        angle = 90

    #if angle < 90:
        #angle -= 1.5
    #elif angle > 90:
        #angle += 1.5

    if angle > 110:
      angle = 110
    elif angle < 70:
      angle = 70


    for i in range(len(c_line)):
        if np.isnan(c_line[0][0]) == False:
            output = cv2.circle(output, (round(c_line[i][0]), round(c_line[i][1])), 3, [255, 255, 255], -1)

    # Display image (flipped) and set up stop key
    outputf = cv2.flip(output,1)
    cv2.imshow("Result", outputf)
    key = cv2.waitKey(10) & 0xFF

    # Save image to file
    saver = RPiCamera.save_frame(outputf, saver)
    
    t1 = time.time()
    tT += t1-t0
    
    # Ramp up motor power
    if tT < t_RAMP:
        pwr = tT*pwr_MAX/t_RAMP
    elif pwr == 0:
        pwr = pwr_MAX/2
    else:
        pwr = pwr_MAX
    
    
    # Send motor and steering commands over CAN network
    motorCAN(pwr, SEND_COMMS)
    steerCAN(angle, SEND_COMMS)

    n_frames += 1

    t2 = time.time()
    tT += t2-t1

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or GPIO.event_detected(21):
    
        print("\nStopping...\n")

        steerCAN(90, SEND_COMMS)
        
        steps = np.floor(pwr/5)
        
        for ts in range(int(steps)):
            motorCAN(5*(steps-ts-1), SEND_COMMS)
            time.sleep(0.25)

        break

    #cv2.waitKey(10)



# Release saved video file
saver.release()
cv2.destroyAllWindows()

# Print time stats
Stats(tT,n_frames)

# Shut down
#print("\nShutting down...\n")  
#os.system("sudo shutdown -h now")
