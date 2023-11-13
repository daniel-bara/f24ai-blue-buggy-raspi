from fileinput import close
import math
import numpy as np
import cv2
from .path_follower_abc import PathFollowerABC


LOOK_AHEAD_DIST = 3
LOOK_AHEAD_RADIUS = 300
SAMPLE_DENSITY = 100
pi = math.pi

hfv = 31.1*(pi/180)
vfv = 24.4*(pi/180)
h = 0.125
W = 0.5

class PathFollowing:
    def __init__(self):
        super().__init__()
    
    def get_steer(self, path, look_ahead_radius, sample_density, c):

        if not path:
            # Make speed == 0 here
            return

        def PointsInArc(r, n):

            #V = 0.5 - h/(2*r*np.cos(hfv*(2*x/n-1))*np.tan(vfv))
            #H = 0.5*(1 + np.tan(hfv*(2*x/n-1))/np.tan(hfv))

            return [[ 0.5*(1 + np.tan(hfv*(2*x/n-1))/np.tan(hfv)) , 0.5 + h/(2*r*np.cos(hfv*(2*x/n-1))*np.tan(vfv))] for x in range(0, n+1)]

        def TrueSteering(theta, r, W): # Compute required steer angle from radial
            # position of target (theta,r) and wheelbase of vehicle (W) in metres
            
            theta = theta*np.pi/180

            tana = 2*W*np.cos(theta) / ( r + 2*W*np.sin(theta) )

            alpha = np.degrees( np.pi/2 - np.arctan(tana) )

            return alpha

        semi_arc = PointsInArc(look_ahead_radius, sample_density)

        for i in range(len(semi_arc)):
            semi_arc[i][0] = round(semi_arc[i][0]*640)
            semi_arc[i][1] = round(semi_arc[i][1]*480)

        for i in range(len(semi_arc)):
            c = cv2.circle(c, (semi_arc[i][0], semi_arc[i][1]), 3, (0,0,255), -1)

        distance = 0
        lowest_distance = 10000000
        closest_point = [0,0]

        # Find closest points
        for i in range(len(semi_arc)):
            for j in range(len(path)):
                distance = ((((semi_arc[i][0] - path[j][0]) ** 2) + (
                            (semi_arc[i][1] - path[j][1]) ** 2)) ** 0.5)
                if distance < lowest_distance:
                    lowest_distance = distance
                    #closest_point = path[j]
                    closest_point = semi_arc[i]
        

        c = cv2.circle(c, (round(closest_point[0]), round(closest_point[1])), 5, (0,255,0), 2)
        
        theta = 90 - np.arctan((1-2*closest_point[0]/640)*np.tan(hfv))*180/pi

        steer_angle = TrueSteering(theta, look_ahead_radius, W)

        if steer_angle is None:
            steer_angle = 90

        return steer_angle
