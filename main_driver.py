# This script ...

import numpy as np
import cv2
import time
from menu import MainMenu
from delayed_interrupt import DelayedKeyboardInterrupt
from HexConv import steerCAN, motorCAN
from camera.laptop_camera import LaptopCamera
from camera.pi_camera import RPiCamera
from camera_perception import CVCameraPerception
from path_planner import WallCentreLine
# from path_follower.path_follower_update import PathFollowing
from path_follower.arc_path_follower import PathFollowing
import RPi.GPIO as GPIO

def stats(t,i): # Print timing stats following stop command

    print('\n')
    print('Total runtime:         ' + str(round(t,3)) + ' seconds' )
    print('Total frames:          ' + str(i) + ' frames' )
    print('Average frame rate:    ' + str(round(i/t,2)) + ' Hz\n')

def ramp_down(P, com): # Ramp motor power down to zero (and steer->90)

    print("\nStopping...\n")

    steerCAN(90, com)
    
    steps = np.floor(P/5)
    
    for ts in range(int(steps)):
        pwr = 5*(steps-ts-1)
        motorCAN(pwr, com)
        time.sleep(0.25)

SEND_COMMS = 0      # send CAN commands?
ARC_RADIUS = 0.75   # path follower arc radius in m

TESTING_MODE = 0    # terminates program after 'NT' frames when True
NT = 100

# Motor ramp vars
pwr_MAX = 50
t_RAMP = 5

# Set up stop button press recognition
GPIO.setmode(GPIO.BCM)
GPIO.setup(21,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(21,GPIO.FALLING,bouncetime=2000)

# Enter menu
menu = MainMenu()
menu.__init__()
[menu_display, menu_sliders, menu_mode, menu_save, menu_morph] = menu.main_menu()

# Set up CAN interface, set steering to straight
if SEND_COMMS:
    bashCommand = 'sudo ip link set can0 up type can bitrate 500000'
    spc(bashCommand, shell=True)
    steerCAN(90, SEND_COMMS)

# MENU OPTION
# Create class instances
if menu_mode: # if in laptop mode
    camera = LaptopCamera(menu_save)
else: # if in PI mode
    camera = RPiCamera(menu_save)

# Initialise video feed
if menu_display:
    blank = np.zeros((camera.RESOLUTION[0],camera.RESOLUTION[1],3),np.uint8)
    cv2.imshow("Result", blank)

# camera perception
cv_perception = CVCameraPerception(menu_sliders, menu_mode, menu_morph) # runs init automatically
cv_perception.__init__(menu_sliders, menu_mode, menu_morph)

# path planner
wall_line = WallCentreLine()
wall_line.__init__()

# path follower
path_follower = PathFollowing()
path_follower.__init__()

# Init vehicle vectors
# vehicle_position, vehicle_forward, vehicle_right = vehicle_vectors
# vehicle vectors set to the approximate position of the front axis of vehicle relative to camera origin
vehicle_vectors = [0,550,320]

# initialise lidar here
#######

# allow the camera & lidar to warmup
time.sleep(0.1)

W = 0.5

pwr = 0
tT = 0
n_frames = 0

T0=time.time()
avgT = 0

while True:
    
    try:

        print(' > Frame ' + str(int(n_frames+1)))

        t0 = time.time()

        with DelayedKeyboardInterrupt(): # allow this code to finish before interrupting
            
            image = camera.get_frame()
            T1=time.time()
            print(T1-T0)
            avgT=(avgT*n_frames+(T1-T0))/(n_frames+1)
        
        [percept_frame, blue_p, yellow_p] = cv_perception.get_cones(image)
        output = percept_frame

        [c_line, meta] = wall_line.path_generator(blue_p, yellow_p, vehicle_vectors)

        angle = path_follower.get_steer(c_line, ARC_RADIUS, 50, image)

        if angle is None:
            angle = 90
        
        for i in range(len(c_line)):
            if np.isnan(c_line[0][0]) == False:
                output = cv2.circle(output, (round(c_line[i][0]), round(c_line[i][1])), 3, [255, 255, 255], -1)

        output = cv2.flip(output,1)

        # MENU OPTIONS
        if menu_display:
            # Display image
            cv2.imshow("Result", output)

        # if menu_save is true, save frame to file
        if menu_save:
            camera.save_frame(output)

        t1 = time.time()
        tR = tT+t1-t0
        
        # Ramp up motor power
        if tR < t_RAMP:
            pwr = tR*pwr_MAX/t_RAMP
        elif pwr == 0:
            pwr = pwr_MAX/2
        else:
            pwr = pwr_MAX
        
        # Format and send motor and steering commands via CAN (togglable)
        motorCAN(pwr, SEND_COMMS)
        steerCAN(angle, SEND_COMMS)

        # Iterate frame number
        n_frames += 1

        t2 = time.time()
        tT += t2-t0

        # if the `q` key was pressed, break from the loop
        key = cv2.waitKey(10) & 0xFF
        if key == ord("q") or GPIO.event_detected(21):

            ramp_down(pwr,SEND_COMMS)

            break

        if TESTING_MODE and n_frames==NT:
            break

        #cv2.waitKey(10)
        T0=time.time()

    
    except KeyboardInterrupt: # CTRL-C will break the loop to terminate program properly

        ramp_down(pwr,SEND_COMMS)

        break

# Release saved video file
cv2.destroyAllWindows()

print(round(avgT,3))

# Print time stats
stats(tT,n_frames)

# Shut down
#print("\nShutting down...\n")  
#os.system("sudo shutdown -h now")
