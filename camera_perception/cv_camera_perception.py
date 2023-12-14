from .camera_perception_abc import CameraPerceptionABC
import numpy as np
import cv2

# Smoothing Factor
S_F = 5


class CVCameraPerception(CameraPerceptionABC):

    # Menu options
    IS_SLIDER = False
    COLOUR_MODE = False
    MORPHOLOGY = False # unused

    # Morphology Kernel
    SIZE = 51
    KERNEL = [] # Kernel set in __INIT__

    # Default hsv values
    # PI camera @ idx[0]. Laptop camera @ idx[1]
    # Blue
    HLowB = [32,32]
    HHighB = [125,125]
    SLowB = [183,141]
    SHighB = [255,167]
    VLowB = [87,141]
    VHighB = [255,255]
    # Yellow
    HLowY = [25,25]
    HHighY = [42,42]
    SLowY = [111,111]
    SHighY = [255,255]
    VLowY = [50,50]
    VHighY = [255,255]

    def __init__(self, sliders, mode, morph):
        super().__init__()

        # Menu options
        self.COLOUR_MODE = mode
        self.IS_SLIDER = sliders
        self.MORPHOLOGY = morph

        # Morphology Kernel
        if morph == True:
            self.KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.SIZE,self.SIZE))
        else:
            self.KERNEL = np.zeros((self.SIZE, self.SIZE), np.uint8)
            self.KERNEL[0:self.SIZE, self.SIZE//2] = 1

        if sliders == True:

            # idk why (but it's to do with the nothing in createtrackbar)
            def nothing(x):
                pass

            cv2.namedWindow('Blue Cones')
            cv2.createTrackbar('Hue Low','Blue Cones',self.HLowB[mode],360,nothing)
            cv2.createTrackbar('Hue High','Blue Cones',self.HHighB[mode],360,nothing)
            cv2.createTrackbar('Sat Low','Blue Cones',self.SLowB[mode],255,nothing)
            cv2.createTrackbar('Sat High','Blue Cones',self.SHighB[mode],255,nothing)
            cv2.createTrackbar('Val Low','Blue Cones',self.VLowB[mode],255,nothing)
            cv2.createTrackbar('Val High','Blue Cones',self.VHighB[mode],255,nothing)
            cv2.namedWindow('Yellow Cones')
            cv2.createTrackbar('Hue Low','Yellow Cones',self.HLowY[mode],360,nothing)
            cv2.createTrackbar('Hue High','Yellow Cones',self.HHighY[mode],360,nothing)
            cv2.createTrackbar('Sat Low','Yellow Cones',self.SLowY[mode],255,nothing)
            cv2.createTrackbar('Sat High','Yellow Cones',self.SHighY[mode],255,nothing)
            cv2.createTrackbar('Val Low','Yellow Cones',self.VLowY[mode],255,nothing)
            cv2.createTrackbar('Val High','Yellow Cones',self.VHighY[mode],255,nothing)

    def get_cones(self, frame):
        """
        Find cones within certain colour range and return them in the world frame
        """
        # Smooth the image to account for fuzz
        # frame = cv2.filter2D(frame, -1, np.ones((S_F, S_F), np.float32)/(S_F**2))

        # Convert bgr frame to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        [BLUE_RANGE, YELLOW_RANGE] = self.get_range()

        # Mask frame with limits
        blue = cv2.inRange(hsv, *BLUE_RANGE)
        yellow = cv2.inRange(hsv, *YELLOW_RANGE)

        # Close vertical masks to define full cones
        blue_cone = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, self.KERNEL)
        yellow_cone = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, self.KERNEL)

        # Find contours on the mask
        b_contours, _ = cv2.findContours(
            blue_cone, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        y_contours, _ = cv2.findContours(
            yellow_cone, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        blue_contours = [(cnt, 'b') for cnt in b_contours]
        yellow_contours = [(cnt, 'y') for cnt in y_contours]

        # Draw the contours on the frame
        frame = cv2.drawContours(frame, b_contours, -1, (0, 255, 0), 3)
        frame = cv2.drawContours(frame, y_contours, -1, (255, 0, 255), 3)

        b_points, y_points = [], []

        # Magic number: The rough size of a cone at a distance (pixels)
        area_threshold = 30

        for cnt, colour in blue_contours + yellow_contours:
            # Only include contours that have at least a certain area
            area = cv2.contourArea(cnt)
            if area < area_threshold:
                continue

            # Calculate contour centre of mass
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])

            # Approximate a polygon to the contour
            approx = cv2.approxPolyDP(
                cnt, 0.009 * cv2.arcLength(cnt, True), True)

            # Locate the highest pixel y-coordinate
            cy = max(c[0][1] for c in approx)

            # Draw dot on contour center
            frame = cv2.circle(frame, (cx, cy), 3, [0, 0, 255], -1)

            if colour == 'b':
                b_points.append((cx, cy, area))
            elif colour == 'y':
                y_points.append((cx, cy, area))

        return frame, b_points, y_points

    def get_range(self):
        
        mode = self.COLOUR_MODE

        if self.IS_SLIDER == True:

            # Uses local variables (e.g., HLowB not self.HLowB)
            # Read trackbar positions
            #Blue
            HLowB = cv2.getTrackbarPos('Hue Low','Blue Cones')
            HHighB = cv2.getTrackbarPos('Hue High','Blue Cones')
            SLowB = cv2.getTrackbarPos('Sat Low','Blue Cones')
            SHighB = cv2.getTrackbarPos('Sat High','Blue Cones')
            VLowB = cv2.getTrackbarPos('Val Low','Blue Cones')
            VHighB = cv2.getTrackbarPos('Val High','Blue Cones')
            #Yellow
            HLowY = cv2.getTrackbarPos('Hue Low','Yellow Cones')
            HHighY = cv2.getTrackbarPos('Hue High','Yellow Cones')
            SLowY = cv2.getTrackbarPos('Sat Low','Yellow Cones')
            SHighY = cv2.getTrackbarPos('Sat High','Yellow Cones')
            VLowY = cv2.getTrackbarPos('Val Low','Yellow Cones')
            VHighY = cv2.getTrackbarPos('Val High','Yellow Cones')
            #Create arrays
            BlueLow = np.array([HLowB,SLowB,VLowB])
            BlueHigh = np.array([HHighB,SHighB,VHighB])
            YellowLow = np.array([HLowY,SLowY,VLowY])
            YellowHigh = np.array([HHighY,SHighY,VHighY])

            # Mask frame with limits
            BLUE_RANGE = [BlueLow,BlueHigh]
            YELLOW_RANGE = [YellowLow,YellowHigh]
        else:
            # Default HSV Cone colour ranges
            ### Colour range values as tested on F22 cones
            BLUE_RANGE = [np.array([self.HLowB[mode], self.SLowB[mode], self.VLowB[mode]]), np.array([self.HHighB[mode], self.SHighB[mode], self.VHighB[mode]])]
            YELLOW_RANGE = [np.array([self.HLowY[mode], self.SLowY[mode], self.VLowY[mode]]), np.array([self.HHighY[mode], self.SHighY[mode], self.VHighY[mode]])]


        return BLUE_RANGE, YELLOW_RANGE