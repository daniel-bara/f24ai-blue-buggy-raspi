import numpy as np
import sys
from subprocess import call as spc

def hexify(val):
    
    out = '{:02x}'.format(int(val))
    
    return out

def motorCAN(val, a):

    # Convert from percentage to integer decimal range 0-1950
    
    hexa = round(abs(val)*19.5, 0)
    
    H1 = hexify(hexa % 256)
    H2 = hexify(hexa // 256)
    
    bashCommand = ['cansend can0 522#' + H1 + H2 + '000000000000']
    
    if a:
        spc(bashCommand, shell=True)

    print('(1314)  Motor -> ', '{0:.1f}'.format(val), ' %')
    
    out = [H1, H2, 0, 0, 0, 0, 0, 0]
    
    return out

def steerCAN(val, a):

    # Convert from range 0-180 to integer decimal range 0-1800
    
    hexa = round(abs(val)*10, 0)
    
    H1 = hexify(hexa % 256)
    H2 = hexify(hexa // 256)
    
    bashCommand = ['cansend can0 523#' + H1 + H2 + '000000000000']

    if a:
        spc(bashCommand, shell=True)

    print('(1315)  Steer -> ', '{0:.1f}'.format(val), ' degrees')
    
    out = [H1, H2, 0, 0, 0, 0, 0, 0]
    
    return out
