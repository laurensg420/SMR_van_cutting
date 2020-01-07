# %%
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

import van_robot
import time, datetime
import operator

class RobotSteps( object ):
    def __init__(self):
        self.world_handler = plot_3d_sympy.DigitalWorldHandler()
        # self.world_handler.plot()
        self.steps = {
            0: {
                "desc": "Move to beginning position",
                "func": self.step0
            },
            1: {
                "desc": "Detect aruco marker",
                "func": self.step1
            },
            2: {
                "desc": "Move to the aruco marker",
                "func": self.step2
            },
            3: {
                "desc": "Measure real y-distance",
                "func": self.step3
            },
            4: {
                "desc": "Move offset of board",
                "func": self.step4
            },
            5: {
                "desc": "Measure distance of left and top flange of the window",
                "func": self.step5
            },
            6: {
                "desc": "Centre to window",
                "func": self.step6_7
            },
            8: {
                "desc": "Move out and move to centre of window",
                "func": self.step8_9
            },
            10: {
                "desc": "Move tool to board",
                "func": self.step10
            },
            11: {
                "desc": "Measure orientation of bars",
                "func": self.step11
            },
            12: {
                "desc": "Create x- and z-axis",
                "func": self.step12
            },
            13: {
                "desc": "Move to origin",
                "func": self.step13
            },
            13.5: {
                "desc": "Move_out",
                "func": self.step135
            },
            14: {
                "desc": "Move to origin",
                "func": self.step14
            },
            15: {
                "desc": "Move to starting position",
                "func": self.step15
            },
            16: {
                "desc": "ask for initiation cut",
                "func": self.step16
            },
            17: {
                "desc": "cut",
                "func": self.step17
            }
        }
        self.ordered_steps = sorted( self.steps.items(), key=operator.itemgetter( 0 ) )

        self._connected_robot = False
        self._connected_aruco = False
        self._connected_arduino = False
        self._variables_set = False

        self.start_cut = False

    def connect(self, camera_index, arduino_port):
        try:
            a = self.robot_connect()
        except:
            a = False

        try:
            b = self.aruco_connect(camera_index)
        except:
            b = False

        try:
            c = self.arduino_connect(arduino_port)
        except:
            c = False
        return a, b, c
        # return self.robot_connect(), self.aruco_connect(camera_index), self.arduino_connect(arduino_port)

    def robot_connect(self):
        try:
            time.sleep(1)
            print( 'connected to robot', datetime.datetime.now() )
            self._connected_robot = True
            return True
        except:
            print("Couldn't connect to robot", datetime.datetime.now() )
            self._connected_robot = False
            return False

    def aruco_connect(self, camera_index):
        try:
            aruco.connect(camera_index)
            print( 'aruco connected', datetime.datetime.now() )
            self._connected_aruco = True
            return True
        except:
            print( 'aruco not connected', datetime.datetime.now() )
            self._connected_aruco = False
            return False

    def arduino_connect(self, port):
        try:
            # only run once
            if not self._connected_arduino:
                time.sleep(1)
                print( 'arduino connected', datetime.datetime.now() )
            self._connected_arduino = True
            return True
        except:
            self._connected_arduino = False
            return False

    def aruco_list_cameras(self):
        return 1, [[]]

    def arduino_list_ports(self):
        class Dummy(object):
            def __init__(self):
                self.device = "COM8"
        l = [Dummy()]
        return l

    def set_window(self, window_placement_x, window_placement_z, max_window_size_x, max_window_size_z):
        print( 'setup variables', datetime.datetime.now() )
        # settings variables
        self.window_placement_x, self.window_placement_z = window_placement_x, window_placement_z
        self.max_window_size_x, self.max_window_size_z = max_window_size_x, max_window_size_z

        self.hori_length = self.max_window_size_x  # distance in meters
        self.vert_length = self.max_window_size_z  # distance in meters
        self.radius = 0.06
        self.x_axis_location = self.window_placement_x  # [m]
        self.z_axis_location = self.window_placement_z  # [m]

        self._variables_set = True
        print( 'end of step', datetime.datetime.now() )

    def get_robot_joints(self):
        self.rob.getj()

    @property
    def initialized(self):
        return self._connected_arduino and self._connected_aruco and self._connected_robot and self._variables_set

    def step0(self):
        if not self.initialized:
            raise Exception("First connect to all devices and set all variables!")

        time.sleep(1)

        print( 'move to beginning position', datetime.datetime.now() )

    def step1(self):
        if not self.initialized:
            raise Exception("First connect to all devices and set all variables!")
        print( 'step 1', datetime.datetime.now() )
        # step 1 - aruco measure
        time.sleep(3)
        print( 'end of step', datetime.datetime.now() )

    def step2(self):
        print( 'step 2', datetime.datetime.now() )
        # # step 2 - move to aruco location with offset of 0.2 meters
        time.sleep(2)

        print( 'end of step', datetime.datetime.now() )

    def step3(self):
        print( 'step 3', datetime.datetime.now() )
        time.sleep(3)
        print( 'end of step', datetime.datetime.now() )

    def step4(self):
        print( 'step 4', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )

    def step5(self):
        print( 'step 5', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )

    def step6_7(self):
        # %%

        print( 'step 6/7', datetime.datetime.now() )
        time.sleep(1)

        print( 'end of step', datetime.datetime.now() )

    def step8_9(self):
        # %%

        print( 'step 8/9', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )

        # %%

    def step10(self):
        print( 'step 10', datetime.datetime.now() )
        time.sleep(1)

        print( 'end of step', datetime.datetime.now() )

        # %%

    def step11(self):
        print( 'step 11', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )

        # %%

    def step12(self):
        print( 'step 12', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )

    def step13(self):
        print( 'step 13', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )

        # %%

    def step135(self):
        print( 'step 13.5', datetime.datetime.now() )
        time.sleep(1)

        print( 'end of step', datetime.datetime.now() )

    def step14(self):
        # %%

        print( 'step 14', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )

    def step15(self):
        # %%

        print( 'step 15', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )

    def step16(self):
        start_cut = input( "Is this the right starting position for the window cutout? Yes/No ", )
        if start_cut == "Yes" or start_cut == "yes" or start_cut == "y":
            self.start_cut = True
        else:
            print( "You disapproved the startpoint. Cut will not be executed" )

    def step17(self):
        # if self.start_cut:
        print( 'step 17', datetime.datetime.now() )
        time.sleep(1)
        print( 'end of step', datetime.datetime.now() )


if __name__ == "__main__":
    stepHandler = RobotSteps()
    device_count, imgs = stepHandler.aruco_list_cameras()
    camera_index = device_count-1
    port_list = stepHandler.arduino_list_ports()
    stepHandler.set_window(0.20, 0.20, 0.10, 0.05)
    stepHandler.connect(camera_index, port_list[0].device)
    for key, step in stepHandler.ordered_steps:
        print('key', key)
        print(step['desc'])
        print(step['func']())
