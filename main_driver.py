# This script ...

import time
import os
import numpy as np
from numpy import pi
import cv2
import sys
from menu import MainMenu
from tools.delayed_interrupt import DelayedKeyboardInterrupt
from tools.control_request import ControlRequest
from camera.pi_camera import RPiCamera
from camera.laptop_camera import LaptopCamera
from camera_perception import CVCameraPerceptionSlide
from path_planner import WallCentreLine
from path_follower.arc_path_follower import PathFollowing
from RPi import GPIO
# NOTE: importing above takes ~7seconds

from picamera.array import PiRGBArray
from picamera import PiCamera

def stats(camera, t,i): # Print timing stats following stop command

    RES = camera.res

    rt = str(round(t,3))
    f = str(i)
    fr = str(round(i/t,2))

    txt1 = 'Total runtime:          ' + rt + ' seconds'
    txt2 = 'Total frames:           ' + f + ' frames'
    txt3 = 'Average frame rate:     ' + fr + ' Hz'

    print('\n' + txt1 + '\n' + txt2 + '\n' + txt3 + '\n')

    if camera.save:

        final_frame = np.zeros((RES[1],RES[0],3), np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX

        txt_col = (0,255,0)

        final_frame = cv2.putText(final_frame, txt1, (10,int(RES[1]*2/5)), font, 0.6, txt_col, 1, cv2.LINE_AA)
        final_frame = cv2.putText(final_frame, txt2, (10,int(RES[1]*1/2)), font, 0.6, txt_col, 1, cv2.LINE_AA)
        final_frame = cv2.putText(final_frame, txt3, (10,int(RES[1]*3/5)), font, 0.6, txt_col, 1, cv2.LINE_AA)

        for _ in range(3):

            camera.save_frame(final_frame)


if __name__ == "__main__":

    VERTICAL_RES = 360
    ARC_DISTANCE = 0.6
    FNAME = ''

    UNPLUG_ETH = 0
    TESTING_MODE = 0
    NT = 100

    SEND_REQS = [1,1] # [motor,steer]

    # Motor ramp vars
    pwr_MAX = 20
    t_RAMP = 2

    # Set up button press recognition
    channel = 16
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(channel,GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(channel,GPIO.FALLING,bouncetime=1000)

    # Quick input toggle (-d for defaults | -v for video ON + defaults)
    if np.size(sys.argv,0)>1:
        menu_arg = sys.argv[1]
    else:
        menu_arg=0

    # Enter menu
    menu = MainMenu()
    [menu_display, menu_sliders, menu_mode, menu_save, menu_morph] = menu.main_menu(menu_arg)

    # Initialise class objects
    cv_perception = CVCameraPerceptionSlide(menu_sliders, menu_mode, menu_morph)
    wall_line = WallCentreLine()
    path_follower = PathFollowing()
    control_request = ControlRequest(SEND_REQS,t_RAMP,pwr_MAX)

    # 4:3 approximate aspect ratio
    if VERTICAL_RES == 360:
        RES = [480,368] # avoid restructuring warnings for this specific res
    else:
        RES = [int(VERTICAL_RES*4/3),VERTICAL_RES]

    # Create class instance, get frame to warm up
    if menu_mode: # if in laptop mode
        camera = LaptopCamera(RES, menu_save)
    else: # if in PI mode
        camera = RPiCamera(RES, menu_save, name=FNAME, pwr=pwr_MAX, r=ARC_DISTANCE)

    # Temp iterator
    i = 0

    # Init vehicle vectors (vehicle_position, vehicle_forward, vehicle_right)
    vehicle_vectors = [0, RES[1]*1.2, RES[0]/2]

    # Initialise video feed
    if menu_display:
        blank = np.ones((RES[1],RES[0],3), np.uint8)
        cv2.imshow("Result",blank)

    tT = 0
    n_frames = 0

    # Waits for ethernet unplug
    if UNPLUG_ETH:

        eth_loc = '/../../sys/class/net/eth0/carrier'
        loop = 0

        while int(open(eth_loc, 'r').read()):
            # ^file reads 1 when eth plugged in, 0 when not

            if loop == 0:
                print('\nWaiting for ethernet to be unplugged...')
                loop = 1

    # Then wait for button press to begin execution
    print('\nWaiting for button press...')
    while True:

        if GPIO.event_detected(channel):
            break

    time.sleep(1)

    t0 = time.time()
    control_request.reset_timer()

    # MAIN LOOP
    try: # for 'CTRL-C' handler

        while True:

            print(' > Frame ' + str(int(n_frames+1)))

            with DelayedKeyboardInterrupt(): # allow this code to finish before interrupting
                
                # Collect image data
                image = camera.get_frame()

            # Identify cones within scene from camera
            [percept_frame, blue_p, yellow_p] = cv_perception.get_cones(image) # *time consuming
            output = percept_frame

            [c_line, meta] = wall_line.path_generator(blue_p, yellow_p, vehicle_vectors)

            # Calculate steering angle
            angle = path_follower.get_steer_arc(c_line, ARC_DISTANCE, 50, image)
            # angle = path_follower.get_steer_direct(c_line, image)

            # Display path line on image
            for i in range(len(c_line)):
                if np.isnan(c_line[0][0]) == False:
                    output = cv2.circle(output, (round(c_line[i][0]), round(c_line[i][1])), 3, [255, 255, 255], -1)

            # Display image and set up stop key
            if menu_display:
                cv2.imshow("Result", output) # *time consuming

            # Save image to file
            if menu_save:
                camera.save_frame(output)

            # Ramp up motor power on each frame until constant
            pwr = control_request.ramp_up_step()

            # Send steering request
            control_request.send_steer(angle)

            n_frames += 1

            t1 = time.time()
            tT += t1-t0

            # if the `q` key was pressed, break from the loop
            key = cv2.waitKey(10) & 0xFF
            if key == ord("q"):
                break

            # Check for 1st button press
            if GPIO.event_detected(channel):

                print('\n*HOLD MODE*')

                # Immediately stop vehicle ('HOLD' mode)
                control_request.ramp_down(pwr)

                while True:

                    # 2nd press -> continue execution ('RESTART' mode) unless 3rd press detected
                    if GPIO.event_detected(channel):

                        # Hold time
                        time.sleep(1)

                        # Still pressed -> shut system down ('STOP' mode)
                        if GPIO.input(channel) == GPIO.LOW:

                            print("\n*TERMINATING EXECUTION...*") 

                            # Twitch to visibly indicate termination
                            control_request.twitch()

                            # Before exit, print time stats
                            stats(camera,tT,n_frames)

                            # Shutdown or terminate program
                            # os.system("sudo shutdown -h now")
                            sys.exit()

                        # If not held press, restart
                        print('\n*RESTARTING...*')

                        # Before continuing, print time stats and reset vars
                        stats(camera, tT, n_frames)
                        tT = 0
                        n_frames = 0

                        time.sleep(2)

                        # Reset timer to allow speed to ramp up again
                        control_request.reset_timer()

                        break # out of button loop, NOT main loop

                    # Limit checking to 20Hz
                    time.sleep(1/20)

            # if in testing, break after NT frames
            if TESTING_MODE and n_frames==NT:
                break

            t0 = time.time()


    except KeyboardInterrupt:
        pass


    # Ramp down speed, steer to 90
    control_request.ramp_down(pwr)

    # Show time stats
    stats(camera,tT,n_frames) 

    # Shut down
    #print("\nShutting down...\n")  
    #os.system("sudo shutdown -h now")
