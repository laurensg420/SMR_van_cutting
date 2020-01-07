import van_robot
import time
import copy
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sympy import *
from sympy.solvers import solve
from sympy import Symbol
import arduino_serial

# Declare the I/O numbers
switch_X = 1
switch_Y = 2
switch_Z = 0
i = 1

# Declare size window and the IR stopping distance
delta_x = 0.2
delta_z = -0.3
x_off = 0.1
z_off = -0.2
safe_US_distance = 0.3

# Declare the port and baudrate for the Arduino
# arduino = arduino_serial.Arduino("COM8")
# distance = arduino.distance
# duration = arduino.duration

rob = van_robot.Robot("192.168.0.1", True)  # Set IP address
orig_tcp = (0, 0, 0.135, 0, 0, 0)
rob.set_tcp(orig_tcp)  # Set Tool Center Point
rob.set_payload(0.5, (0, 0, 0))  # Set payload for joints
time.sleep(0.2)  # leave some time to robot to process the setup commands
rob.set_orientation((1,0,0))
# start position of the robot
# assumed is a robot start position which is close to the centre of the measured plane
pos_start = rob.getl()

rob.movel(pos_start)
print("Current tool pose is: ", rob.getl())
#
# rob.measure_triangle(arduino, 0.05)
rob.movel(pos_start)