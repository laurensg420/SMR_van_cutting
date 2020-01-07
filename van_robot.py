import urx
import time
import copy
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from sympy.vector import CoordSys3D
from sympy.solvers import solve
from sympy import Symbol
import arduino_serial

# from sympy.vector import CoordSys3D
# from sympy.vector import express
# from sympy.plotting import *
import sympy.vector
import sympy
import sympy.abc

from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib as plt
import random

import plot_3d_sympy


class Plane( object ):
    def __init__(self):
        self.a1 = None
        self.b = None
        self.c = None
        self.d = None
        self.theta = None

        self.point1 = None
        self.point2 = None
        self.point3 = None

    def set_points(self, p1, p2, p3):
        self.point1 = p1
        self.point2 = p2
        self.point3 = p3

    def plane_calc(self):
        # only take the translation part of the locations
        p1 = np.array( self.point1[0:3] )
        p2 = np.array( self.point2[0:3] )
        p3 = np.array( self.point3[0:3] )

        # These two vectors are in the plane
        v1 = p3 - p1
        v2 = p2 - p1

        # the cross product is a vector normal to the plane
        cp = np.cross( v1, v2 )
        a1, b, c = cp

        # This evaluates a * x3 + b * y3 + c * z3 which equals d
        d = np.dot( cp, p3 )

        print( 'The equation is {0}x + {1}y + {2}z = {3}'.format( a1, b, c, d ) )

        # Calculate angle between table and van
        # Set Z to 0 to get the XY-plane
        # 'The equation is {0}x + {1}y + = {3}'

        # # Calculate length of Y by setting X to 0
        # y = Symbol('y')
        # # A1*x + B*y + C*z - D = 0
        # y = solve(self.b * y - self.d, y)
        # y_length = math.fabs(float(y[0]))
        # # print('Length (Y) is ', y_length)
        #
        # # Calculate X by setting Y to 0
        # x = Symbol('x')
        # # A1*x + B*y + C*z - D = 0
        # x = solve(self.a1 * x - self.d, x)
        # x_length = math.fabs(float(x[0]))
        # # print('Length (X) is ', x_length)
        #
        # theta = math.atan(y_length / x_length)
        # theta = math.degrees(theta)
        # theta = 90 + theta
        # print('Angle between table and van is ', theta)
        # return theta

        self.a1 = a1
        self.b = b
        self.c = c
        self.d = d

        # self.theta = theta

    def plot(self):
        fig = pyplot.figure()
        ax = Axes3D( fig )

        ax.scatter( 0, 0, 0, label='Robot position' )
        x, y, z = self.point1[0:3]
        ax.scatter( x, y, z, label='Point 1' )
        x, y, z = self.point2[0:3]
        ax.scatter( x, y, z, label='Point 2' )
        x, y, z = self.point3[0:3]
        ax.scatter( x, y, z, label='Point 3' )

        ax.legend()
        ax.set_xlabel( "x-axis" )
        ax.set_ylabel( "y-axis" )
        ax.set_zlabel( "z-axis" )

        yy, xx = np.meshgrid( range( 3 ), range( 2 ) )

        xx = xx - 0.09
        yy = yy - 0.35
        zz = yy * 0 - 0.15
        ax.plot_surface( xx, yy, zz, color='burlywood', label="Table" )
        pyplot.show()

    def deviation_calc(self, pos):
        y = Symbol( 'y' )
        # A1*x + B*y + C*z - D = 0
        y = solve( self.a1 * pos[0] + self.b * y + self.c * pos[2] - self.d, y )
        # print('y-value is ', y)
        return y[0]


class Robot( urx.Robot ):
    def __init__(self, host, use_rt):
        super().__init__( host, use_rt )

        self.cal_points = []
        self.tcp = (0, 0, 0, 0, 0, 0)
        self.plane = Plane()

        self.side_points = []

    def move_y(self, y_pos):
        cur_pos_y = super().getl()
        cur_pos_y[1] = y_pos

        x, y, z, rx, ry, rz = cur_pos_y
        super().movel( (x, y, z, rx, ry, rz), a, v )

    def move_y_plane(self, y_offset=0, position=None):
        if position is None:
            position = super().getl()
        if y_offset > 0:
            raise Exception( "Can't move robot through plane" )

        y = self.plane.deviation_calc( position )
        y += y_offset
        position[1] = y

        super().movel( position, 0.1, 0.1 )

    def move_radius(self, radius, corner_no, y_offset, clockwise=True, current_pos=True, acc=0.01, vel=0.01):
        if corner_no == 0 or corner_no == 1:
            delta_x_r = radius
        else:
            delta_x_r = -radius

        if corner_no == 0 or corner_no == 3:
            delta_z_r = radius
        else:
            delta_z_r = -radius

        cur_pos_r = super().getl()
        pose_t = copy.deepcopy( cur_pos_r )
        pose_t[0] += delta_x_r
        pose_t[2] += delta_z_r

        half_sqrt_2 = 0.7071067811865475
        pose_v = copy.deepcopy( cur_pos_r )
        # pose_v[0] += half_sqrt_2 * delta_x

        if not corner_no % 2:  # if 0 or 2
            pose_v[0] += (1 - half_sqrt_2) * delta_x_r
            pose_v[2] += half_sqrt_2 * delta_z_r
        else:  # if 1 or 3
            pose_v[0] += half_sqrt_2 * delta_x_r
            pose_v[2] += (1 - half_sqrt_2) * delta_z_r

        super().movec( pose_v, pose_t, acc, vel )

    def move_radius_digital_world(self, radius, corner_no, y_offset, world: plot_3d_sympy.DigitalWorldHandler):
        """
        Corners are defined as followed
            ---> x
         |  0 --- 1
         |  |     |
         |  |     |
        \_/ 3 --- 2
        z

        0 : top left corner
        1 : top right corner
        2 : bottom right corner
        3 : bottom left corner

        :param radius: the size of the radius
        :param corner_no:
        :param y_offset:
        """
        if 3 < corner_no or corner_no < 0:
            raise Exception( "expected corner_no argument between 0 and 3" )
        print( 'h1' )
        # calculating pose to
        if corner_no == 0 or corner_no == 1:
            delta_x_r = radius
        else:
            delta_x_r = -radius

        if corner_no == 0 or corner_no == 3:
            delta_z_r = -radius
        else:
            delta_z_r = radius
        #
        world.setl( self.getl() )
        world.world.align_tcp_to_robot()
        world.new_world()
        world.move_tcp( (delta_x_r, delta_z_r, 0) )
        world.move_tcp_to_plane( y_offset )
        pose_to = world.getl()
        #

        # calculating pose via
        half_sqrt_2 = 0.7071067811865475
        if not corner_no % 2:  # if 0 or 2
            delta_via_x = (1 - half_sqrt_2) * delta_x_r
            delta_via_z = half_sqrt_2 * delta_z_r
        else:  # if 1 or 3
            delta_via_x = half_sqrt_2 * delta_x_r
            delta_via_z = (1 - half_sqrt_2) * delta_z_r

        #
        world.setl( self.getl() )
        world.world.align_tcp_to_robot()
        world.new_world()
        world.move_tcp( (delta_via_x, delta_via_z, 0) )
        world.move_tcp_to_plane( y_offset )
        pose_via = world.getl()
        #
        super().movec( pose_via, pose_to )  # , acc, vel)

    def move_horizontal(self, hor_length, orientation, y_offset):
        if orientation == 'right':
            cur_pos = super().getl()
            cur_pos[0] += hor_length

            self.move_y_plane( y_offset, cur_pos )

        if orientation == 'left':
            cur_pos = super().getl()
            cur_pos[0] -= hor_length

            self.move_y_plane( y_offset, cur_pos )

    def move_vertical(self, ver_length, orientation, y_offset):
        if orientation == 'down':
            cur_pos = super().getl()
            cur_pos[2] -= ver_length

            self.move_y_plane( y_offset, cur_pos )

        if orientation == 'up':
            cur_pos = super().getl()
            cur_pos[2] += ver_length

            self.move_y_plane( y_offset, cur_pos )

    def move_offset_origin(self, x_axis_offset, z_axis_offset, y_offset=-0.1):
        cur_pos = super().getl()
        cur_pos[0] += x_axis_offset
        cur_pos[2] += z_axis_offset
        self.move_y_plane( y_offset, cur_pos )

    def move_offset_digital_world(self, world: plot_3d_sympy.DigitalWorldHandler, x_axis_offset, z_axis_offset,
                                  y_offset=0.15, origin=False, acc=0.01, vel=0.01):
        if origin:
            if len( origin ) != 2:
                raise Exception( "Expected iterable of length 2 (origin x-axis position, z-axis position)" )
            cur_pos = self.getl()
            cur_pos[0] = origin[0]
            cur_pos[2] = origin[1]
            world.setl( cur_pos )
            world.move_tcp_to_plane( 0.15 )
        else:
            world.setl( self.getl() )
        world.world.align_tcp_to_robot()
        world.new_world()
        world.move_tcp( (x_axis_offset, z_axis_offset, 0) )
        world.move_tcp_to_plane( y_offset )
        self.movel( world.getl(), acc, vel )

    def move_to_begin_window(self, world, hor_length, ver_length, radius, origin, y_offset=0.1, x_axis_offset=0,
                                  z_axis_offset=0, mill=False, acc=0.1, vel=0.1):
        # origin_offset
        self.move_offset_digital_world( world, (x_axis_offset + radius), z_axis_offset, y_offset=0.15,
                                        origin=origin, acc=acc, vel=vel )
        self.move_offset_digital_world( world, (x_axis_offset + radius), z_axis_offset, y_offset=y_offset,
                                        origin=origin, acc=acc, vel=vel )

    def draw_square_digital_world(self, world, hor_length, ver_length, radius, origin, y_offset=0.1, x_axis_offset=0,
                                  z_axis_offset=0, mill=False, acc=0.01, vel=0.01):
        if mill == False and y_offset <= 0:
            y_offset = 0.030
        print( "Cut has been initiated." )
        if mill:
            self.set_digital_out( 1, True )
        # move in
        self.move_offset_digital_world( world, 0, 0, y_offset, acc=acc, vel=vel )
        time.sleep(0.5)
        # horizontal
        self.move_offset_digital_world( world, hor_length, 0, y_offset, acc=acc, vel=vel )
        # 1st bend
        self.move_radius_digital_world( radius, 1, y_offset, world )
        # -vertical
        self.move_offset_digital_world( world, 0, ver_length, y_offset, acc=acc, vel=vel )
        # 2nd bend
        self.move_radius_digital_world( radius, 2, y_offset, world)
        # -horizontal
        self.move_offset_digital_world( world, -hor_length, 0, y_offset, acc=acc, vel=vel )
        # 3th bend
        self.move_radius_digital_world( radius, 3, y_offset, world )
        # vertical
        self.move_offset_digital_world( world, 0, -ver_length, y_offset, acc=acc, vel=vel )
        # Final bend
        self.move_radius_digital_world( radius, 0, y_offset, world )
        # move out
        self.move_offset_digital_world( world, 0, 0, 0.10, acc=acc, vel=vel )

        if mill:
            self.set_digital_out( 1, False )

    def draw_square(self, hor_length, ver_length, radius, y_offset=-0.1, x_axis_offset=0, z_axis_offset=0):
        # origin_offset
        self.move_offset_origin( (x_axis_offset + radius), z_axis_offset, y_offset )
        start_cut = input( "Is this the right starting position for the window cutout? Yes/No ", )
        if start_cut == "Yes" or "yes" or "y":
            print( "Cut has been initiated." )
            # horizontal
            self.move_horizontal( (hor_length), 'right', y_offset )
            # 1st bend
            self.move_radius( radius, 1, y_offset )
            # -vertical
            self.move_vertical( ver_length, 'down', y_offset )
            # 2nd bend
            self.move_radius( radius, 2, y_offset )
            # -horizontal
            self.move_horizontal( hor_length, 'left', y_offset )
            # 3th bend
            self.move_radius( radius, 3, y_offset )
            # vertical
            self.move_vertical( ver_length, 'up', y_offset )
            # Final bend
            self.move_radius( radius, 0, y_offset )
        else:
            print( "You disapproved the startpoint. Cut will not be executed" )

    def centre_for_triangle(self, device: arduino_serial.Arduino, measurement_offset=-0.03, board_distance=-0.10):
        # get current ditance from the side of the van
        aruco_offset = device.get_multiple_average( device.get_distance_top, 100 ) / 1000  # [m]
        aruco_offset -= 0.008  # correct for 8 mm of tcp offset of ultrasonic sensor

        # set offsets for measurements
        set_centre_measurement = aruco_offset + measurement_offset
        set_triangle_safe_dis = aruco_offset + board_distance

        # do measurements
        self.translate( (0, set_centre_measurement, 0, 0, 0, 0), 0.1, 0.1 )
        self.move_joint( 5, 45, 0.1, 0.5 )
        time.sleep( 0.2 )
        centre_check_1 = device.get_multiple_average( device.get_distance_side, 100 ) / 1000
        self.move_joint( 5, 135, 0.1, 0.5 )
        time.sleep( 0.2 )
        centre_check_2 = device.get_multiple_average( device.get_distance_side, 100 ) / 1000
        x_to_centre = 0.30 - centre_check_1
        z_to_centre = centre_check_2 - 0.28
        self.translate( (0, -(set_triangle_safe_dis + measurement_offset), 0, 0, 0, 0), 0.1, 0.3 )
        self.translate( (x_to_centre, 0, z_to_centre, 0, 0, 0), 0.1, 0.1 )

    def measure_triangle(self, device: arduino_serial.Arduino, translation: float, acc=0.01,
                         vel=0.01):  # translation in [m]

        measure_points = (
            (translation, 0, translation),
            (0, 0, -2 * translation),
            (-2 * translation, 0.0, translation)
        )

        start_pos = super().getl()
        tool_orient = [0, -2.22, -2.22]
        start_pos[3] = tool_orient[0]
        start_pos[4] = tool_orient[1]
        start_pos[5] = tool_orient[2]
        super().movel( start_pos, acc, vel )

        for point in measure_points:
            super().translate( point, acc, vel )
            super().translate_tool()
            dis = device.get_multiple_average( device.get_distance_top, 100 ) / 1000  # [m]
            dis -= 0.008  # correct for 8 mm of tcp offset of ultrasonic sensor

            # calculate the real position
            cur_pos = super().getl()
            # cur_pos[1] = cur_pos[1] - self.tcp[2]
            cur_pos[1] = cur_pos[1] + dis

            self.cal_points.append( cur_pos )

        super().movel( start_pos, acc, vel )

        p1, p2, p3 = self.cal_points
        self.plane.set_points( p1, p2, p3 )
        self.plane.plane_calc()

    def measure_triangle_tool(self, device: arduino_serial.Arduino,
                              world: plot_3d_sympy.DigitalWorldHandler,
                              translation: float,
                              acc=0.01,
                              vel=0.01):  # translation in [m]

        measure_points = (
            (translation, -translation,),
            (0, 2 * translation,),
            (-2 * translation, -translation, 0)
        )
        self.cal_points = []
        for point in measure_points:
            # super().translate(point, acc, vel)
            super().translate_tool( point, acc, vel )
            dis = device.get_multiple_average( device.get_distance_top, 100 ) / 1000  # [m]
            # dis -= 0.008  # correct for 8 mm of tcp offset of ultrasonic sensor
            world.setl( super().getl() )
            world.set_ultrasonic_top_distance( dis )
            cur_pos = world.getl_ultrasonic_top()

            self.cal_points.append( cur_pos )

        # super().movel(start_pos, acc, vel)
        world.world.set_plane( self.cal_points )
        p1, p2, p3 = self.cal_points
        self.plane.set_points( p1, p2, p3 )
        self.plane.plane_calc()

    def set_tcp(self, tcp):
        super().set_tcp( tcp )
        self.tcp = tcp

    def move_joint(self, joint_index, degrees, acc=0.1, vel=0.05):
        # print(degrees)
        a = super().getj()
        a[joint_index] = math.radians( degrees )
        super().movej( a, acc, vel )

    def measure_sides(self, device: arduino_serial.Arduino, side_list=None):
        if side_list is None:  # default value
            side_list = ["left", "top"]

        orientations = {  # degrees, axis, pos/neg
            "left": [45, "x", -1],  # left
            "top": [45 + 90 * 1, "z", 1],  # top
            "right": [45 + 90 * 2, "x", 1],  # right
            "bottom": [45 + 90 * 3, "z", -1]  # bottom
        }

        # set orientation of tool right
        start_pos = super().getl()
        tool_orient = [0, -2.22, -2.22]
        start_pos[3] = tool_orient[0]
        start_pos[4] = tool_orient[1]
        start_pos[5] = tool_orient[2]
        super().movel( start_pos )

        side_points = []

        for side in side_list:
            degrees, axis, direction = orientations[side]

            self.move_joint( 5, degrees, 0.1, 0.5 )

            distance = device.get_multiple_average( device.get_distance_side, 100 ) / 1000  # [m]
            print( degrees, ' - ', distance )
            # calc action point position
            cur_pos = super().getl()
            if axis == "x":
                index = 0
            elif axis == "y":
                index = 1
            elif axis == "z":
                index = 2
            else:
                raise Exception( "Didn't find index" )
            cur_pos[index] += distance * direction

            side_points.append(
                [side, distance, cur_pos] )

        return side_points

    def side_look(self, device: arduino_serial.Arduino):
        # measure_list = [  # degrees, axis, pos/neg
        #     [45, "x", -1],  # left
        #     [45 + 90 * 1, "z", 1],  # top
        #     [45 + 90 * 2, "x", 1],  # right
        #     [45 + 90 * 3, "z", -1]  # bottom
        # ]
        measure_list = [  # degrees, axis, pos/neg
            [45, "x", -1],  # left
            [45 + 90 * 1, "z", 1],  # top
            [45 + 90 * 2, "x", 1],  # right
            [45 + 90 * 3, "z", -1]  # bottom
        ]

        cur_pos = super().getl()
        y_pos = self.plane.deviation_calc( cur_pos )
        y_pos2 = y_pos - 0.010
        cur_pos[1] = y_pos2
        super().movel( cur_pos )

        for degrees, axis, direction in measure_list:
            self.move_joint( 5, degrees, 0.1, 0.5 )
            distance = device.get_multiple_average( device.get_distance_side, 100 ) / 1000  # [m]
            print( degrees, ' - ', distance )
            # calc action point position
            cur_pos = super().getl()
            if axis == "x":
                index = 0
            elif axis == "y":
                index = 1
            elif axis == "z":
                index = 2
            else:
                raise Exception( "Didn't find index" )
            cur_pos[index] += distance * direction

            self.side_points.append(
                [degrees, distance, cur_pos] )

        self.move_joint( 5, measure_list[0][0], 0.1, 0.5 )

    def go_to_point(self, x_window, z_window):
        if len( self.side_points ) > 0:
            left_pos = self.side_points[0][2]
            x = left_pos[0] + x_window

            bottom_pos = self.side_points[3][2]
            z = bottom_pos[2] + z_window

            y = self.plane.deviation_calc( [x, 0, z] )
            y -= 0.050

            pos = [x, y, z, 0, -2.22, -2.22]
            print( pos )
            super().movel( pos )

    def compensate_tool(self, joint, theta):
        # Todo check the working of this function

        print( 'Rotate 2nd joint to ', self.theta )
        self.move_joint( joint, self.theta )

    def repetitive_def_axis(self, device: arduino_serial.Arduino, y_offset=-0.03, axis_measurement_offset=0.15):
        '''
        finds four points on the left and top side of the borders of the plane.
        Points found, can be used to define x-axis (top) and z-axis (left).
        :param device: arduino objects
        :param axis_measurement_offset: safety distance from the borders of the plane
        :param y_offset: perpendicular offset from plane
        :return:
        '''
        points1 = []
        points2 = []

        # left 1
        self.move_y_plane( y_offset )
        self.move_joint( 5, 45, 0.1, 0.5 )
        x_check = device.get_multiple_average( device.get_distance_side, 100 ) / 1000
        cur_pos = self.getl()
        cur_pos[0] -= (x_check - axis_measurement_offset)
        self.move_y_plane( y_offset, cur_pos )
        x_offset = device.get_multiple_average( device.get_distance_side, 100 ) / 1000
        cur_pos = self.getl()
        cur_pos[0] -= x_offset
        points1.append( ['left', x_offset, cur_pos] )
        # top 1
        self.move_joint( 5, 135, 0.1, 0.5 )
        z_check = device.get_multiple_average( device.get_distance_side, 100 ) / 1000
        if z_check > 0.4:
            z_check = 0.25
        else:
            z_check
        z_start = self.getl()
        safe_move = (z_check - axis_measurement_offset)
        z_start[2] += (z_check - axis_measurement_offset)
        self.move_y_plane( y_offset, z_start )
        z_offset = device.get_multiple_average( device.get_distance_side, 100 ) / 1000
        cur_pos = self.getl()
        cur_pos[2] += z_offset
        points1.append( ['top', z_offset, cur_pos] )
        # print(points1)
        # left 2
        self.move_joint( 5, 45, 0.1, 0.5 )
        x1_check = device.get_multiple_average( device.get_distance_side, 100 ) / 1000
        cur_pos = self.getl()
        cur_pos[0] -= x1_check
        points2.append( ['left', x1_check, cur_pos] )
        # top 2
        last_point = self.getl()
        last_point[0] += safe_move
        self.move_y_plane( y_offset, last_point )
        self.move_joint( 5, 135, 0.1, 0.5 )
        z1_offset = device.get_multiple_average( device.get_distance_side, 100 ) / 1000
        cur_pos = self.getl()
        cur_pos[2] += z1_offset
        points2.append( ['top', z1_offset, cur_pos] )
        # print(points2)

        return points1, points2

    def repetitive_def_axis_digital_world(self, device: arduino_serial.Arduino,
                                          world: plot_3d_sympy.DigitalWorldHandler, y_offset=0.03,
                                          axis_measurement_offset=0.15, centre_window_pose=None, acc=0.05, vel=0.5):
        '''
        finds four points on the left and top side of the borders of the plane.
        Points found, can be used to define x-axis (top) and z-axis (left).
        :param device: arduino objects
        :param axis_measurement_offset: safety distance from the borders of the plane
        :param y_offset: perpendicular offset from plane
        :return:
        '''

        points1 = []
        points2 = []
        # left 1
        # self.move_y_plane( y_offset )
        # look to the left
        x_check, x_check_position = self.measure_sides_angle_correction( world, device, acc, vel, side='left' )
        safe_move_x = (x_check - axis_measurement_offset)

        # move to the left with an offset to the edge
        world.setl( x_check_position )
        world.world.align_tcp_to_robot()
        world.move_tcp( (-safe_move_x, 0, 0) )
        world.move_tcp_to_plane( y_offset )
        world.world.rotate_z_tcp( -0.75 * sympy.pi )
        world.plot()
        self.movel( world.getl(), acc, vel )

        # look to the left
        x_offset, x_offset_position = self.measure_sides_angle_correction( world, device, acc, vel, side='left' )

        # calculate robot position of measured point
        world.setl( x_offset_position )
        world.set_ultrasonic_side_distance( x_offset )
        cur_pos = world.getl_ultrasonic_side( convert_to_float=True )
        points1.append( ['left', x_offset, cur_pos] )

        # top 1
        # look at top with side ultrasonic
        z_check, z_check_position = self.measure_sides_angle_correction(world, device, acc, vel, side='top')
        z_start = self.getl()

        safe_move_z = (z_check - axis_measurement_offset)

        # move to the top with an offset
        world.setl( z_check_position )
        world.world.align_tcp_to_robot()
        world.move_tcp( (0, -safe_move_z, 0) )
        world.move_tcp_to_plane( y_offset )
        world.world.rotate_z_tcp( -0.25 * sympy.pi )
        self.movel( world.getl(), acc, vel )

        # look at top
        z_offset, z_offset_position = self.measure_sides_angle_correction( world, device, acc, vel, side='top' )

        # calculate robot position of top
        world.setl( z_offset_position )
        world.set_ultrasonic_side_distance( z_offset )
        cur_pos = world.getl_ultrasonic_side( convert_to_float=True )

        points1.append( ['top', z_offset, cur_pos] )
        print( points1 )

        # left 2
        # look to the left with side ultrasonic
        x1_check, x1_check_position = self.measure_sides_angle_correction( world, device, acc, vel, side='left' )

        world.setl( x1_check_position )
        world.set_ultrasonic_side_distance( x1_check )
        cur_pos = world.getl_ultrasonic_side( convert_to_float=True )
        points2.append( ['left', x1_check, cur_pos] )

        # top 2
        # move to the right
        world.setl( self.getl() )
        world.world.align_tcp_to_robot()
        world.move_tcp( (safe_move_x, 0, 0) )
        world.move_tcp_to_plane( y_offset )
        world.world.rotate_z_tcp( -0.25 * sympy.pi )
        self.movel( world.getl(), acc, vel )

        # look at top
        z1_offset, z1_offset_position = self.measure_sides_angle_correction( world, device, acc, vel, side='top' )

        world.setl( z1_offset_position )
        world.set_ultrasonic_side_distance( z1_offset )
        cur_pos = world.getl_ultrasonic_side( convert_to_float=True )

        points2.append( ['top', z1_offset, cur_pos] )
        print( points2 )

        return points1, points2

    def measure_sides_angle_correction(self, world, arduino, acc=0.01, vel=0.01, side='left', threshold=0.50, measure_distance = False):
        print('start measure side {}, with threshold {}'.format(side, threshold))
        if side == 'left':
            rotation_z = -0.75 * sympy.pi
            direction = +1
        elif side == 'top':
            rotation_z = -0.25 * sympy.pi
            direction = -1
        world.setl( self.getl() )
        world.world.align_tcp_to_robot()
        world.world.rotate_z_tcp( rotation_z )
        self.movel( world.getl(), acc, vel )
        time.sleep( 0.2 )
        cur_pos = self.getl()
        move_back_position = self.getl()

        centre_check_1 = arduino.get_multiple_average( arduino.get_distance_side, 100 ) / 1000
        if centre_check_1 > threshold:
            print( "bigger than 0.5 meter" )
            for angle in range( 2, 10, 1 ):
                world.setl( cur_pos )
                world.world.align_tcp_to_robot()
                if side == 'left':
                    world.world.rotate_y_tcp( direction * angle, angle_in_degrees=True )
                if side == 'top':
                    world.world.rotate_x_tcp( direction * angle, angle_in_degrees=True )
                    world.move_tcp( (0, 0, -0.0005 * angle - 0.01) )
                world.world.rotate_z_tcp( rotation_z )
                world.plot()
                print( 'move to angle: {}'.format( angle ) )
                self.movel( world.getl(), acc, vel )
                print( 'measure again' )
                centre_check_1 = arduino.get_multiple_average( arduino.get_distance_side, 100 ) / 1000
                if centre_check_1 < threshold:
                    break

            if angle == 9 and centre_check_1 > threshold:
                if side == 'top':
                    world.setl( cur_pos )
                    world.world.align_tcp_to_robot()
                    world.move_tcp( (0, -0.05, 0) )
                    self.movel( world.getl(), acc, vel )
                    centre_check_1, cur_pos = self.measure_sides_angle_correction( world, arduino, acc=acc, vel=vel, side=side,
                                                         threshold=threshold )
                    if measure_distance:
                        centre_check_1 += 0.05
                elif side == 'left':
                    world.setl( cur_pos )
                    world.world.align_tcp_to_robot()
                    world.move_tcp( (-0.05,0, 0) )
                    self.movel( world.getl(), acc, vel)
                    centre_check_1, cur_pos = self.measure_sides_angle_correction( world, arduino, acc=acc, vel=vel, side=side,
                                                         threshold=threshold )
                    if measure_distance:
                        centre_check_1 += 0.05
                print( "Didn't find the {} distance".format( side ) )

            print( "{} side bigger than 0.50 meter: {}".format( side, centre_check_1 ) )
        self.movel( move_back_position )
        print('end of measure side {}'.format(side))
        return centre_check_1, cur_pos
