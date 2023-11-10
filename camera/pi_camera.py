## This class allows the user to access the laptop camera directly
## It is designed to be used in the same way as the Raspberry Pi camera to operate between the two simply

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from os.path import exists

class RPiCamera:
    def __init__(self, res, save, name=None, pwr=None, r=None):

        self.res = res
        self.save = save

        # create a class for the laptop camera
        self.camera = PiCamera()

        # define frame size
        self.camera.resolution = self.res
        self.camera.framerate = 30
        self.camera.vflip = True
        self.camera.hflip = True

        # White balance modes (will need to be changed under different lighting conditions)
        # if unset, defaults to 'auto'
        # self.camera.awb_mode = 'sunlight'
        # self.camera.awb_mode = 'cloudy'
        # self.camera.awb_mode = 'tungsten'
	  
        self.rawCapture = PiRGBArray(self.camera)

        if self.save:
            # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'XVID')

            # create filename
            if name is not None:
                f1 = name + '_'
            else:
                f1 = ''

            if pwr is not None:
                f2 = str(int(pwr)) + '%_'
            else:
                f2 = ''

            if r is not None:
                f3 = str(round(r,1)) + 'm'
            else:
                f3 = ''

            fnam = f1 + f2 + f3
            i = 0

            while True:
                # iterate through filename versions if it already exists
                if  i == 0:
                    loc = fnam +'.avi'
                else:
                    loc = fnam + '_' + str(i) +'.avi'
                i+=1

                if not exists(loc):
                    self.saver = cv2.VideoWriter(loc, fourcc, 3, (self.res[0], self.res[1]))
                    break

        # Warm up
        _ = self.get_frame()

    def get_frame(self): # reads the camera to output a frame
        
        self.rawCapture.truncate(0)
        self.camera.capture(self.rawCapture, format="bgr", use_video_port=True)
        frame = self.rawCapture.array

        return frame
	
    def save_frame(self, image): # Saves current frame to file

        # save the frame
        self.saver.write(image)
