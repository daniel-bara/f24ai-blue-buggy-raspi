# This is the parent class for the camera
# Holds all properties and some methods for the PI camera and a Laptop Camera

# from .laptop_camera import LaptopCamera
# from .pi_camera import RPiCamera

import cv2

class CameraBase:
    
    def __init__(self, menu_save):

        # Menu options
        self.IS_SAVE = menu_save

        self.RESOLUTION = [640,480]
        self.FRAMERATE = 1.5

        if menu_save:
            # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.SAVER = cv2.VideoWriter('output.avi', fourcc, self.FRAMERATE, (self.RESOLUTION[0], self.RESOLUTION[1]))


    def save_frame(self, image): # Saves current frame to file

        if self.IS_SAVE:
            # save the frame
            self.SAVER.write(image)
