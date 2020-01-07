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

# Declare the port and baudrate for the Arduino and connect to the Arduino
arduino = arduino_serial.Arduino("COM8")

# connect to the robot and setup the robot
rob = van_robot.Robot("192.168.0.1", True)  # Set IP address
orig_tcp = (0, 0, 0.150, 0, 0, 0)
rob.set_tcp(orig_tcp)  # Set Tool Center Point
rob.set_payload(0.5, (0, 0, 0))  # Set payload for joints
time.sleep(0.2)  # leave some time to robot to process the setup commands
a, v = (0.05, 0.5)

# start position of the robot
# assumed is a robot start position which is close to the centre of the measured plane
pos_start = rob.getl()

rob.measure_triangle(arduino, 0.100, a, v)
# rob.side_look(arduino)
rob.move_y_plane(-0.030)

points = rob.measure_sides(arduino)
cur_pos = rob.getl()
side_margin = 0.100  # [m]
movement = 0.100  # [m]
side, distance, point_pos = points[0]
if distance > (side_margin + movement):
    # move to left / negative x
    cur_pos[0] -= movement
else:
    cur_pos[0] += movement
# move to right / positive x

side, distance, point_pos = points[1]
if distance > (side_margin + movement):
    # move to top / positive z
    cur_pos[2] += movement
else:
    # move to bottom / negative z
    cur_pos[2] -= movement

rob.move_y_plane(-0.030, cur_pos)
#
# points = rob.measure_sides(arduino)

print("Current tool pose is: ", rob.getl())

# rob.go_to_point(0.110, 0.110)
# rob.measure_triangle(arduino, 0.05)
# rob.movel(pos_start)