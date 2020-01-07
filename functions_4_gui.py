import urx
import van_robot
import time

import main_digital_world
from gui import *
import main_digital_world

stepHandler = main_digital_world.RobotSteps()

import arduino_serial
import time, datetime

def checkerboard(obj):
    #TODO import checkerboard code or file
    print("checkerboard stuff")


def calibrate_camera_external(obj):
    # TODO import calibration code or file
    print("calibration stuff")


def starting_robot_external(window_placement_x, window_placement_y, max_window_size_x, max_window_size_y):
    print('Placement X', window_placement_x)
    print('Placement Y', window_placement_y)
    print('Size X', max_window_size_x)
    print('Size Y', max_window_size_y)
    # main_digital_world(window_placement_x, window_placement_y, max_window_size_x, max_window_size_y)


def popup_connect_robot(obj: window):
    stepHandler.robot_connect()
    window.Create_popup(obj, text1="Robot connected")

def connect_robot(obj):
     # Connect UR
    rob = van_robot.Robot("192.168.0.1", True)  # Set IP address
    orig_tcp = (0, 0, 0.150, 0, 0, 0)
    rob.set_tcp(orig_tcp)  # Set Tool Center Point
    rob.set_payload(0.5, (0, 0, 0))  # Set payload for joints
    time.sleep(0.2)  # leave some time to robot to process the setup commands
    a, v = (0.03, 0.03)  # acceleration and movement speed of robot
    rob.movel([0, 0, 0.1, 0, 0, 0], a, v, relative=True)

    # Connect Arduino
    arduino = arduino_serial.Arduino("COM8")
    print('arduino connected', datetime.datetime.now())

    # Connect aruco
    #aruco.connect()
    print('aruco connected', datetime.datetime.now())



def useless(obj):
    print('rickrolled')