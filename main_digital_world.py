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
    def __init__(self, plotting=True):
        self.world_handler = plot_3d_sympy.DigitalWorldHandler(plotting)
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
        self.plotting = plotting

    def set_plotting(self, value):
        self.plotting = bool(value)
        self.world_handler.plotting = bool(value)

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
            self.rob = van_robot.Robot( "192.168.0.1", True )  # Set IP address
            self.orig_tcp = (0, 0, 0.290, 0, 0, 0)
            self.rob.set_tcp( self.orig_tcp )  # Set Tool Center Point
            self.rob.set_payload( 0.6 )  # Set payload for joints
            time.sleep( 0.2 )  # leave some time to robot to process the setup commands
            self.a, self.v = (0.03, 0.03)
            self.pos_start = self.rob.getl()
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
                self.arduino = arduino_serial.Arduino( port )
                print( 'arduino connected', datetime.datetime.now() )
            self._connected_arduino = True
            return True
        except:
            self._connected_arduino = False
            return False

    def aruco_list_cameras(self):
        return aruco.list_cameras()

    def arduino_list_ports(self):
        return arduino_serial.Arduino.list_ports()

    def set_window(self, window_placement_x, window_placement_z, max_window_size_x, max_window_size_z):
        print( 'setup variables', datetime.datetime.now() )
        # settings variables
        self.radius = 0.06
        self.window_placement_x, self.window_placement_z = window_placement_x, window_placement_z
        self.max_window_size_x, self.max_window_size_z = max_window_size_x-2*self.radius, max_window_size_z-2*self.radius

        self.hori_length = self.max_window_size_x  # distance in meters
        self.vert_length = self.max_window_size_z  # distance in meters
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
        # step 0 - move to beginning position
        # beginning_joint_position = \
        #     [0.12069842764711754,
        #     -1.702580712362753,
        #     -1.8529905516280962,
        #     0.17940012672474973,
        #     -3.2032719501964775,
        #     -2.632514246116477]
        beginning_joint_position = \
            [2.7217340585456054,
             -1.6760175532240984,
             2.2263889080908497,
             -3.6838368955623118,
             0.42821717007022453,
             -2.3796814138461913]

        print( 'move to beginning position', datetime.datetime.now() )
        self.rob.movej( beginning_joint_position, self.a, self.v * 3 )

    def step1(self):
        if not self.initialized:
            raise Exception("First connect to all devices and set all variables!")
        print( 'step 1', datetime.datetime.now() )
        # step 1 - aruco measure
        self.arduino.extend()
        rvec, tvec = aruco.detect_marker( False )
        print( rvec, tvec )

        # set aruco location in digital world
        self.world_handler.setl( self.rob.getl() )
        self.world_handler.world.set_camera_to_tcp()
        self.world_handler.set_aruco_marker( rvec, tvec )
        self.world_handler.plot()
        aruco.camera.release()
        print( 'end of step', datetime.datetime.now() )

    def step2(self):
        print( 'step 2', datetime.datetime.now() )
        # # step 2 - move to aruco location with offset of 0.2 meters
        self.world_handler.move_to_marker()
        self.world_handler.world.align_tcp_to_robot()
        self.world_handler.world.rotate_z_tcp( -0.75 * sympy.pi )
        self.world_handler.move_tcp( (0, 0, -0.10) )
        self.world_handler.plot()
        self.rob.movel( self.world_handler.getl(), self.a, self.v )

        print( 'end of step', datetime.datetime.now() )

    def step3(self):
        print( 'step 3', datetime.datetime.now() )
        # step 3 - measure "real" y-distance with ultrasonic
        # extend ultrasonic arm
        self.arduino.extend()
        print( 'extend' )
        time.sleep( 1 )

        self.aruco_offset = self.arduino.get_multiple_average( self.arduino.get_distance_top, 100 ) / 1000  # [m]
        j = 0
        while self.aruco_offset > 0.12 and j > 4:
            self.aruco_offset = self.arduino.get_multiple_average( self.arduino.get_distance_top, 100 ) / 1000  # [m]
            if self.aruco_offset <= 0.12:
                break
            j += 1
        if j == 5:
            raise Exception( "Sanity Check Error: Didn't find distance larger than 0.12 m" )
        print( self.aruco_offset )
        print( 'end of step', datetime.datetime.now() )

    def step4(self):
        print( 'step 4', datetime.datetime.now() )
        # step 4 - move 3 cm from the board with respect to ultrasonic measurement
        # set offsets for measurements
        set_centre_measurement = self.aruco_offset - 0.04
        set_triangle_safe_dis = self.aruco_offset + -0.10

        self.world_handler.setl( self.rob.getl() )
        self.world_handler.move_tcp( (0, 0, set_centre_measurement) )

        pose = self.world_handler.getl()
        print( 'move to pose:', pose )
        self.rob.movel( pose, self.a / 2, self.v / 2 )

        print( 'end of step', datetime.datetime.now() )

        cur_pos = self.rob.getl()

        self.rob.movel( cur_pos, self.a, self.v )

    def step5(self):
        print( 'step 5', datetime.datetime.now() )
        # step 5 - measure distance side and top
        # look at left with side ultrasonic

        cur_pos = self.rob.getl()
        self.centre_check_1, _ = self.rob.measure_sides_angle_correction( self.world_handler, self.arduino, self.a, self.v,
                                                                  side='left' ,measure_distance=True) # todo: moving to the top ignored

        # look at top with side ultrasonic
        self.rob.movel( cur_pos, self.a, self.v )

        # move out in z-direction of tcp a bit
        self.world_handler.setl(self.rob.getl())
        self.world_handler.world.align_tcp_to_robot()
        self.world_handler.move_tcp((0, 0, -0.02))
        self.centre_check_2, _ = self.rob.measure_sides_angle_correction( self.world_handler, self.arduino, self.a, self.v,
                                                                  side='top', measure_distance=True) # todo: moving to the top ignored

        self.x_to_centre = 0.30 - self.centre_check_1
        self.z_to_centre = self.centre_check_2 - 0.33
        # todo: implement centre_check_1 and 2 into step 11z
        print( 'end of step', datetime.datetime.now() )

    def step6_7(self):
        # %%

        print( 'step 6/7', datetime.datetime.now() )
        # # step 6 and 7 joined - safe distance of 0.10 meters and move to calculated centre of window
        self.world_handler.setl( self.rob.getl() )
        self.world_handler.world.align_tcp_to_robot()
        self.world_handler.plot()
        self.world_handler.move_tcp( (0, 0, -0.10) )
        self.world_handler.move_tcp( (self.x_to_centre, -self.z_to_centre, 0) )
        self.world_handler.plot()
        pose = self.world_handler.getl()
        print( 'move to pose:', pose )
        self.rob.movel( pose, self.a, self.v )
        self.centre_window_pose = pose

        print( 'end of step', datetime.datetime.now() )

    def step8_9(self):
        # %%

        print( 'step 8/9', datetime.datetime.now() )
        # step 8 - measure triangle
        # step 9 - create virtual plane
        self.start_pos_step_8 = self.rob.getl()
        self.rob.movel( self.start_pos_step_8, self.a, self.v )
        self.rob.measure_triangle_tool( self.arduino, self.world_handler, 0.140, self.a, self.v )
        self.rob.movel( self.start_pos_step_8, self.a, self.v )
        print( 'end of step', datetime.datetime.now() )

        # %%

    def step10(self):
        print( 'step 10', datetime.datetime.now() )
        # step 10 - tool offset board
        self.rob.movel( self.start_pos_step_8, self.a, self.v )
        self.world_handler.setl( self.rob.getl() )
        self.world_handler.plot()
        self.world_handler.move_tcp_to_plane( 0.03 + 0.04 )
        self.world_handler.plot()
        self.rob.movel( self.world_handler.getl(), self.a, self.v )

        print( 'end of step', datetime.datetime.now() )

        # %%

    def step11(self):
        print( 'step 11', datetime.datetime.now() )
        # step 11 corner move
        pose = self.rob.getl()
        self.points1, self.points2 = self.rob.repetitive_def_axis_digital_world( self.arduino, self.world_handler,
                                                                       y_offset=0.01 + 0.04,
                                                                       axis_measurement_offset=0.15, centre_window_pose=self.centre_window_pose, acc=self.a,
                                                                       vel=self.v )
        print( 'move to marker' )
        # world.move_tcp((0,0,-0.2))
        print( 'moved to new position' )
        self.world_handler.plot()

        print( 'end of step', datetime.datetime.now() )

        # %%

    def step12(self):
        print( 'step 12', datetime.datetime.now() )
        # step 12 - create x-axis and z-axis
        # SIDE Coordinates [z-formula]
        self.coor1 = [self.points1[0][2][0], self.points1[0][2][2]]  # [X,Z]
        self.coor2 = [self.points2[0][2][0], self.points2[0][2][2]]

        # TOP Coordinates [x-formula]
        self.coor3 = [self.points1[1][2][0], self.points1[1][2][2]]
        self.coor4 = [self.points2[1][2][0], self.points2[1][2][2]]

        print( 'end of step', datetime.datetime.now() )

    def step13(self):
        print( 'step 13', datetime.datetime.now() )
        # step 13 - create origin
        # create_origin.formula_line('x',self.coor1,self.coor2)
        # create_origin.formula_line('z',self.coor3,self.coor4)
        self.line_x = create_origin.formula_line( 'x', self.coor1, self.coor2 )
        self.line_z = create_origin.formula_line( 'z', self.coor3, self.coor4 )
        self.origin = create_origin.intersect_lines( self.line_x, self.line_z )
        print( self.origin )  # [X,Z]

        print( 'end of step', datetime.datetime.now() )

        # %%

    def step135(self):
        print( 'step 13.5', datetime.datetime.now() )
        # step 13.5 - move out
        self.world_handler.setl( self.rob.getl() )
        self.world_handler.move_tcp( (0, 0, -0.15) )
        self.rob.movel( self.world_handler.getl() )

        print( 'end of step', datetime.datetime.now() )

    def step14(self):
        # %%

        print( 'step 14', datetime.datetime.now() )
        # step 14 - move to origin
        self.arduino.collapse()
        self.rob.move_offset_digital_world( self.world_handler, 0, 0, y_offset=0.15, origin=self.origin, acc=self.a,
                                            vel=self.v )

        print( 'end of step', datetime.datetime.now() )

    def step15(self):
        # %%

        print( 'step 15', datetime.datetime.now() )
        # step 15,16,17 - move to desired offset, safety check, cut/draw

        self.arduino.collapse()
        self.rob.move_to_begin_window( self.world_handler, self.hori_length, self.vert_length, self.radius,
                                            self.origin, y_offset=0.030, x_axis_offset=self.x_axis_location,
                                            z_axis_offset=self.z_axis_location,
                                            acc=self.a, vel=self.v)
        print( 'end of step', datetime.datetime.now() )

    def step16(self):
        start_cut = input( "Is this the right starting position for the window cutout? Yes/No ", )
        if start_cut == "Yes" or start_cut == "yes" or start_cut == "y":
            self.start_cut = True
        else:
            print( "You disapproved the startpoint. Cut will not be executed" )

    def step17(self):

        # if self.start_cut:
        self.rob.draw_square_digital_world( self.world_handler, self.hori_length, self.vert_length, self.radius,
                                                self.origin, y_offset=-0.020, mill=True, x_axis_offset=self.x_axis_location,
                                                z_axis_offset=self.z_axis_location
                                                )


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
