# Ramps motor power down from given starting point
# e.g. to ramp down from 20% power:
# ~/Documents/f22 $ python tools/stop.py 20

from control_request import ControlRequest
import numpy as np
import sys
import time


if __name__ == "__main__":

	print("\nStopping...\n")

	bashCommand = 'sudo ip link set can0 up type can bitrate 500000'
    os.system(bashCommand)

	control_request = ControlRequest([1,1],0,0)

	P = float(sys.argv[1])

	steps = np.floor(P/5)

	for ts in range(int(steps)):
	    pwr = 5*(steps-ts-1)
	    control_request.send(pwr,90)
	    time.sleep(0.25)
