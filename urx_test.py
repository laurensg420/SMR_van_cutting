import urx
import time
import copy
import van_robot

rob = van_robot.Robot("192.168.0.1", True)  # Set IP address
orig_tcp = (0, 0, 0.150, 0, 0, 0)
rob.set_tcp(orig_tcp)  # Set Tool Center Point
rob.set_payload(0.5, (0, 0, 0))  # Set payload for joints
time.sleep(0.2)  # leave some time to robot to process the setup commands
a, v = (0.05, 0.5)  # acceleration and movementspeed of robot

# pos_start = rob.getl()
# rob.movel(pos_start, a, v)

delta_x = 0.100  # distance in meters
delta_z = 0.100  # distance in meters
radius = 0.02

robot = van_robot.Robot

rob.draw_square(delta_x, delta_z, radius)

# van_robot.Robot.draw_rectangle()
