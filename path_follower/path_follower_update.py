from fileinput import close
import math
import cv2
from .path_follower_abc import PathFollowerABC


LOOK_AHEAD_DIST = 3
LOOK_AHEAD_RADIUS = 300
SAMPLE_DENSITY = 100
pi = math.pi

class PathFollowing:
    def __init__(self):
        super().__init__()

    

    def get_steer(self, path, look_ahead_radius, sample_density, c):

        if not path:
            # Make speed == 0 here
            return

        def PointsInCircum(r, n):
            return [[math.cos(2 * pi / n * x) * r, math.sin(2 * pi / n * x) * r] for x in range(0, n + 1)]

        semi_circle = PointsInCircum(look_ahead_radius, sample_density)

        for i in range(len(semi_circle)):
            semi_circle[i][0] = round(semi_circle[i][0] + 320)
            semi_circle[i][1] = round(semi_circle[i][1] + 480)

        for i in range(len(semi_circle)):
            c = cv2.circle(c, (semi_circle[i][0], semi_circle[i][1]), 3, (0,0,255), -1)

        distance = 0
        lowest_distance = 10000000
        closest_point = [0,0]

        # Find closest points
        for i in range(len(semi_circle)):
            for j in range(len(path)):
                distance = ((((semi_circle[i][0] - path[j][0]) ** 2) + (
                            (semi_circle[i][1] - path[j][1]) ** 2)) ** 0.5)
                if distance < lowest_distance:
                    lowest_distance = distance
                    closest_point = path[j]
        

        c = cv2.circle(c, (round(closest_point[0]), round(closest_point[1])), 5, (0,255,0), 2)
        
        angle = math.degrees(math.atan((closest_point[0] - 320) / (480 - closest_point[1]))) + 90

        return angle
