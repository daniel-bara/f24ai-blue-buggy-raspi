from gps import *
import time
import numpy as np
from numpy import pi


def getPositionData(gps):
    nx = gpsd.next()
    if nx['class'] == 'TPV':
        latitude = getattr(nx,'lat', "Unknown")
        longitude = getattr(nx,'lon', "Unknown")
        # print("lon = " + str(longitude) + ", lat = " + str(latitude))
    else:
        longitude=None
        latitude=None

    return longitude, latitude

def LLtoXY(lon,lat,origin):

    radius = 6317000

    x = radius*(lon-origin[0])*(pi/180)
    y = radius*(lat-origin[1])*(pi/180)

    print("x=" + str(round(x,6)) + ", y=" + str(round(y,6)))

    return [x,y]


gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
running=True

ix=0
while running:
    
    lon,lat = getPositionData(gpsd)

    if lon is not None:
        ix+=1
        if ix==1:
            origin=[lon,lat]
            print(origin)
        relxy = LLtoXY(lon,lat,origin)
