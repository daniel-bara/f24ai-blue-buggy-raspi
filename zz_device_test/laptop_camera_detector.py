import numpy as np
import cv2
import time

# idk why (but it's to do with the nothing in createtrackbar)
def nothing(x):
    pass

# HSV Cone colour ranges trackbar
###BLUE_RANGE = [np.array([115, 120, 120]), np.array([130, 255, 255])]
###YELLOW_RANGE = [np.array([25, 50, 50]), np.array([40, 255, 255])]
cv2.namedWindow('Blue Cones')
cv2.createTrackbar('Hue Low','Blue Cones',115,360,nothing)
cv2.createTrackbar('Hue High','Blue Cones',130,360,nothing)
cv2.createTrackbar('Sat Low','Blue Cones',120,255,nothing)
cv2.createTrackbar('Sat High','Blue Cones',255,255,nothing)
cv2.createTrackbar('Val Low','Blue Cones',120,255,nothing)
cv2.createTrackbar('Val High','Blue Cones',255,255,nothing)
cv2.namedWindow('Yellow Cones')
cv2.createTrackbar('Hue Low','Yellow Cones',25,360,nothing)
cv2.createTrackbar('Hue High','Yellow Cones',40,360,nothing)
cv2.createTrackbar('Sat Low','Yellow Cones',50,255,nothing)
cv2.createTrackbar('Sat High','Yellow Cones',255,255,nothing)
cv2.createTrackbar('Val Low','Yellow Cones',50,255,nothing)
cv2.createTrackbar('Val High','Yellow Cones',255,255,nothing)

# init hsv values
HLowB = HHighB = SLowB = SHighB = VLowB = VHighB = HLowY = HHighY = SLowY = SHighY = VLowY = VHighY = 0

# Morphology Kernel
SIZE = 51
KERNEL = np.zeros((SIZE, SIZE), np.uint8)
KERNEL[0:SIZE, SIZE//2] = 1

# Smoothing Factor
S_F = 5


class CV_camera_test:
    def __init__(self):
        super().__init__()

    def get_cones(self, frame):
        """
        Find cones within certain colour range and return them in the world frame
        """
        # Smooth the image to account for fuzz
        frame = cv2.filter2D(frame, -1, np.ones((S_F, S_F), np.float32)/(S_F**2))

        # Convert bgr frame to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
        blue = cv2.inRange(hsv, *[BlueLow,BlueHigh])
        yellow = cv2.inRange(hsv, *[YellowLow,YellowHigh])

        # Close vertical masks to define full cones
        blue_cone = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, KERNEL)
        yellow_cone = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, KERNEL)

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


    # START

# initialize the camera and grab a reference to the raw camera capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 250)

if not cap.isOpened():
    raise IOError("Cannot open webcam")

# initialise the image detection code
test = CV_camera_test()
test.__init__()

# allow the camera to warmup
time.sleep(0.1)

while True:
     
    frame = ret, frame = cap.read()

    image = frame

    [output, blue_p, yellow_p] = test.get_cones(image)

    show = np.hstack([image,output])

    cv2.imshow("Result", show)

    key = cv2.waitKey(10) & 0xFF


    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

    cv2.waitKey(10)