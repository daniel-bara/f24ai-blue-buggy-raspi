from fileinput import close
import time
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
h = 0.128
W = 0.35

class PathFollowing:

    def __init__(self):
        super().__init__()
    
    def get_steer_arc(self, path, look_ahead_radius, sample_density, c):
        '''
        Computes steering angle.
        Uses look ahead circle to compute path follower point and find its angular position.
        '''

        if not path:
            # Make speed == 0 here
            return 90

        def PointsInArc(r, n):

            #V = 0.5 - h/(2*r*np.cos(hfv*(2*x/n-1))*np.tan(vfv))
            #H = 0.5*(1 + np.tan(hfv*(2*x/n-1))/np.tan(hfv))

            return [[ 0.5*(1 + np.tan(hfv*(2*x/n-1))/np.tan(hfv)) , 0.5 + h/(2*r*np.cos(hfv*(2*x/n-1))*np.tan(vfv))] for x in range(0, n+1)]

        def TrueSteering(theta, r, W): # Compute required steer angle from radial
            # position of target (theta,r) and wheelbase of vehicle (W) in metres
            
            theta = (90+theta)*pi/180

            tana = 2*W*np.cos(theta) / ( r + 2*W*np.sin(theta) )

            alpha = np.degrees( np.pi/2 - np.arctan(tana) )

            return alpha

        RES = [0,0]
        RES[0] = np.size(c,1)
        RES[1] = np.size(c,0)

        semi_arc = PointsInArc(look_ahead_radius, sample_density)

        for i in range(len(semi_arc)):
            semi_arc[i][0] = round(semi_arc[i][0]*RES[0])
            semi_arc[i][1] = round(semi_arc[i][1]*RES[1])

        for i in range(len(semi_arc)):
            c = cv2.circle(c, (semi_arc[i][0], semi_arc[i][1]), 3, (255,0,255), -1)

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
        
        theta = np.arctan((1-2*closest_point[0]/RES[0])*np.tan(hfv))*180/pi

        steer_angle = TrueSteering(theta, look_ahead_radius, W)

        font = cv2.FONT_HERSHEY_SIMPLEX
        txt = 'steer angle = ' + str(round(steer_angle,1)) + ' deg'
        c = cv2.putText(c, txt, (10,RES[1]-10), font, 0.5, (0,255,200), 1, cv2.LINE_AA)

        return steer_angle

    def get_steer_direct(self, path, c):
        '''
        Computes steering angle.
        Differs from get_steer_arc as does not use a look ahead circle, but instead infers
        radial position from image data (based on mapping to the ground plane).
        **LOGIC TO DECIDE ON PATH FOLLOWER POINT NEEDS CHANGING IF THIS FUNCTION WILL BE USED
        '''

        if not path:
            # Make speed == 0 here
            return 90

        def TrueSteering(theta, r, W): # Compute required steer angle from radial
            # position of target (theta,r) and wheelbase of vehicle (W) in metres
            
            theta = (90+theta)*pi/180

            tana = 2*W*np.cos(theta) / ( r + 2*W*np.sin(theta) )

            alpha = np.degrees( np.pi/2 - np.arctan(tana) )

            return alpha

        RES = [0,0]
        RES[0] = np.size(c,1)
        RES[1] = np.size(c,0)

        distance = 0
        lowest_distance = 10000000
        closest_point = [0,0]

        # Find closest points
        # THIS LOGIC WILL NEED CHANGING IF GET_STEER_DIRECT IS TO BE USED
        for j in range(len(path)):
            distance = path[j][1]
            if distance < lowest_distance:
                closest_point = path[j]

        c = cv2.circle(c, (round(closest_point[0]), round(closest_point[1])), 5, (255,0,255), 2)
        
        Hp = closest_point[0]/RES[0]
        Vp = (RES[1]-closest_point[1])/RES[1]

        L = h/((1-2*Vp)*np.sin(vfv))
        B = (1-2*Hp)*L*np.sin(hfv)

        look_ahead_radius = np.sqrt( B**2 + L**2 )

        theta = np.arctan(B/L)*180/pi

        steer_angle = TrueSteering(theta, look_ahead_radius, W)

        N = 12
        new_path = np.zeros((N+1,2))

        dis = h/np.sin(vfv)
        alpha = steer_angle-90
        R = W/np.sin(alpha*pi/180)

        for p in range(N-2):

            p+=1
            phi = np.radians((p/N)*2*(theta-alpha)+alpha)
            L = R*np.sin(phi) - W

            if L<dis:
                continue

            B = W/np.tan(alpha*pi/180) - R*np.cos(phi)

            V = 2*L*np.sin(vfv)
            H = 2*L*np.sin(hfv)

            new_path[p][0] = (0.5 - B/H)*RES[0]
            new_path[p][1] = RES[1]*(1-(0.5 - h/V))

            if np.any(np.isnan(new_path[p])):
                print(theta)
                print(alpha)
                print(B)
                print(H)
                print(V)

            c = cv2.circle(c, (int(new_path[p][0]), int(new_path[p][1])), 3, (255,0,255), -1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        txt = 'distance from point = ' + str(round(look_ahead_radius,2)) + 'm, steer angle = ' + str(round(steer_angle,1)) + 'deg'
        c = cv2.putText(c, txt, (10,RES[1]-10), font, 0.5, (0,255,200), 1, cv2.LINE_AA)

        return steer_angle