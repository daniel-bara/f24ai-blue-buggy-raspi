## This class allows the user to access the laptop camera directly
## It is designed to be used in the same way as the Raspberry Pi camera to operate between the two simply

import cv2
import numpy as np
from .camera_base import CameraBase
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

class RPiCamera(CameraBase):

    def __init__(self,menu_save):

        # Run __init__ for the parent class
        super().__init__(menu_save)

        # create a class for the laptop camera
        self.camera = PiCamera()

        # define frame size
        self.camera.resolution = (self.RESOLUTION[0], self.RESOLUTION[1])
        self.camera.framerate = self.FRAMERATE
        self.camera.vflip = True
	  
        self.rawCapture = PiRGBArray(self.camera)


    def get_frame(self): # reads the camera to output a frame
        
        self.rawCapture.truncate(0)
        self.camera.capture(self.rawCapture, format="bgr", use_video_port=True)
        
        # seems slightly different in pidriver
        frame = self.rawCapture.array
	  
        return frame
