


import aruco

import plot_3d_sympy

import numpy as np
import sympy
import arduino_serial
import math
# Robot imports
import van_robot
import cv2
import time
# Create origin
import create_origin

# Calibration imports
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
# from sympy import *
from sympy import Symbol
import arduino_serial
from tqdm import tqdm

import van_robot
import time, datetime

# only run once
arduino = arduino_serial.Arduino( "COM8" )
print('arduino connected', datetime.datetime.now())

while True:
    print('step 3', datetime.datetime.now())
    # step 3 - measure "real" y-distance with ultrasonic
    arduino.extend()
    # extend ultrasonic arm
    print('extend')
    aruco_offset = arduino.get_multiple_average( arduino.get_distance_top, 100 ) / 1000  # [m]
    print(aruco_offset)
    print('end of step', datetime.datetime.now())
    arduino.collapse()
    time.sleep(1)
