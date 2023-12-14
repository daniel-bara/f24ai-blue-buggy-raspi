## This class allows the user to access the laptop camera directly
## It is designed to be used in the same way as the Raspberry Pi camera to operate between the two simply
## It contains functions to save recorded video to file

from .camera_base import CameraBase
import cv2
import numpy as np

class LaptopCamera(CameraBase):

    def __init__(self, menu_save): # Initialises laptop camera

        # Run __init__ for the parent class
        super().__init__(menu_save)

        # create a class for the laptop camera
        self.CAP = cv2.VideoCapture(0)

        # define frame size
        self.CAP.set(cv2.CAP_PROP_FRAME_WIDTH, self.RESOLUTION[0])
        self.CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, self.RESOLUTION[1])

        # verify functionality
        if not self.CAP.isOpened():
            raise IOError("Cannot open webcam")

    def get_frame(self): # reads the camera to output a frame

        frame = ret, frame = self.CAP.read()

        return frame
