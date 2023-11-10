import numpy as np
import sys
from subprocess import call as spc
import time


class ControlRequest():

    def __init__(self, opts, t_ramp, p_max):

        self.opts = opts
        self.t_ramp = t_ramp
        self.P_max = p_max

        self.steer_offset = 0 # to correct servo zero error (may invalidate wheel angle mapping)

        # Set up CAN network
        if np.any(opts):
            bashCommand = 'sudo ip link set can0 up type can bitrate 500000'
            spc(bashCommand, shell=True)
            self.send(0,90)

        self.T0 = time.time()

    def reset_timer(self):

        self.T0 = time.time()

    def hexify(self,val):
        
        out = '{:02x}'.format(int(val))
        
        return out

    def send(self, pwr, ang):

        self.send_motor(pwr)
        self.send_steer(ang)

    def send_motor(self, pwr):

        # Convert from percentage to integer range 0-1950
        pval = round(abs(pwr)*19.5, 0)

        P1 = self.hexify(pval % 256)
        P2 = self.hexify(pval // 256)

        pwr_com =   ['cansend can0 512#' + P1 + P2 + '000000000000']

        if self.opts[0]:
            spc(pwr_com, shell=True)

        print('(1298)  Motor -> ', '{0:.1f}'.format(pwr), ' %')

    def send_steer(self, ang):

        # Convert from float range 0-180 to integer range 0-1800
        sval = round(abs(ang+self.steer_offset)*10, 0)

        S1 = self.hexify(sval % 256)
        S2 = self.hexify(sval // 256)

        steer_com = ['cansend can0 513#' + S1 + S2 + '000000000000']

        if self.opts[1]:
            spc(steer_com, shell=True)

        print('(1299)  Steer -> ', '{0:.1f}'.format(ang), ' degrees')


    def ramp_up_step(self): # Ramp up motor power each frame

        t = time.time() - self.T0

        if t < self.t_ramp:
            self.pwr = t*self.P_max/self.t_ramp

        else:
            self.pwr = self.P_max

        self.send_motor(self.pwr)

        return self.pwr

    def ramp_down(self,P): # Ramp motor power down to zero (and steer->90)

        if P == 0:
            return

        print("\nStopping vehicle...\n")

        T = 1.5    # ramp down time
        steps = 8  # number of steps to ramp down over
        
        for i in range(steps):
            pwr = P*(1-(i+1)/steps)
            self.send_motor(pwr)
            self.send_steer(90)
            time.sleep(T/steps)

    def twitch(self, amp=10):

        self.send_steer(90-amp)
        time.sleep(0.2)
        self.send_steer(90+amp)
        time.sleep(0.2)
        self.send_steer(90)
