from .camera_perception_abc import CameraPerceptionABC
import numpy as np
import cv2

# HSV Cone colour ranges
#BLUE_RANGE = [np.array([32, 80, 141]), np.array([125, 104, 255])]
#YELLOW_RANGE = [np.array([25, 111, 50]), np.array([42, 255, 255])]

# Morphology Kernel
SIZE = 51
KERNEL = np.zeros((SIZE, SIZE), np.uint8)
KERNEL[0:SIZE, SIZE//2] = 1

# Smoothing Factor
S_F = 5


class CVCameraPerception(CameraPerceptionABC):
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

        # Mask frame with limits
        blue = cv2.inRange(hsv, *BLUE_RANGE)
        yellow = cv2.inRange(hsv, *YELLOW_RANGE)

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
