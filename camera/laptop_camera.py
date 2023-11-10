## This class allows the user to access the laptop camera directly
## It is designed to be used in the same way as the Raspberry Pi camera to operate between the two simply
## It contains functions to save recorded video to file

import cv2
import numpy as np

class LaptopCamera:

    def __init__(self,res,save): # Initialises laptop camera

        self.res = res
        self.save = save

        # create a class for the laptop camera
        self.cap = cv2.VideoCapture(0)

        # define frame size
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[0])

        # verify functionality
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")

        if self.save:
            # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.saver = cv2.VideoWriter('output.avi', fourcc, 10.0, (self.res[0], self.res[1]))
    

    def get_frame(self): # reads the camera to output a frame

        frame = ret, frame = self.cap.read()

        return frame

    def save_frame(self,image): # Saves current frame to file

        # save the frame
        self.saver.write(image)

        return saver