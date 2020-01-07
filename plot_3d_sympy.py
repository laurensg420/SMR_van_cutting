# partly sourced https://stackoverflow.com/questions/57015852/is-there-a-way-to-plot-a-3d-cartesian-coordinate-system-with-matplotlib
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib

matplotlib.use( 'Qt5Agg' )
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

from sympy.abc import u, v
from mpmath import atan2, degrees, radians

import sympy.vector as sv
from sympy.vector import CoordSys3D, AxisOrienter

from sympy import Plane, Point3D, Line3D
from sympy.vector import express
# from sympy.plotting import
from sympy import Symbol
from sympy import sqrt, ImmutableDenseMatrix, cos, sin
import sympy.vector
import sympy
import cv2

import sympy.matrices
from sympy.matrices import dense
from symbolic_coordsys import CoordSysVariable, DummyVariable


class Arrow3D( FancyArrowPatch ):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__( self, (0, 0), (0, 0), *args, **kwargs )
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform( xs3d, ys3d, zs3d, renderer.M )
        self.set_positions( (xs[0], ys[0]), (xs[1], ys[1]) )
        FancyArrowPatch.draw( self, renderer )


class DigitalWorldHandler( object ):
    def __init__(self, plotting = True):
        self.plotting = plotting
        if self.plotting:
            self.fig = plt.figure()
            self.ax1 = self.fig.add_subplot( 111, projection='3d' )
            self.arrow_prop_dict = dict( mutation_scale=20, arrowstyle='->', shrinkA=0, shrinkB=0 )

        self.settings = DigitalWorldSettings()
        self.settings.camera_rvec_wrt_tcp.set( [
            0,  # -0.5 * sympy.pi,
            0,
            0.75 * sympy.pi
        ])
        self.settings.camera_tvec_wrt_tcp.set( [
            0.05303300,
            0.05303300,
            -(0.290 - 0.150)
        ])

        self.settings.tool_head_tvec.set( [
            0,
            0,
            -0.290
        ])
        self.settings.tool_head_rvec.set( [
            0, 0, 0
        ])

        self.settings.ultrasonic_top_tvec.set( [
            0,
            0,
            0.290 + 0.035
        ] )
        self.settings.ultrasonic_top_rvec.set( [
            0,
            0,
            -0.25 * sympy.pi
        ] )

        self.settings.ultrasonic_side_tvec.set( [
            0.048,
            0.0,
            0.290 + 0.035
        ] )
        self.settings.ultrasonic_side_rvec_1.set( [
            0,
            0,
            -0.25 * sympy.pi
        ] )
        self.settings.ultrasonic_side_rvec_2.set( [
            0,
            0.5 * sympy.pi,
            0
        ] )
        self.settings.ultrasonic_top_distance = 0.300
        self.settings.ultrasonic_side_distance = 0.300

        self.world = DigitalWorld( self.settings )

    def set_ultrasonic_top_distance(self, distance):
        self.settings.ultrasonic_top_distance = float( distance )

    def set_ultrasonic_side_distance(self, distance):
        self.settings.ultrasonic_side_distance = float( distance )

    def getl_ultrasonic_top(self, float=False):
        self.new_world()
        a = self.world.getl_ultrasonic_top()
        if float:
            a = dense.matrix2numpy( a, dtype=float )
            a = a.flatten()
        return a

    def getl_ultrasonic_side(self, convert_to_float=False):
        self.new_world()
        a = self.world.getl_ultrasonic_side()
        if convert_to_float:
            a = dense.matrix2numpy( a, dtype=float )
            a = a.flatten()
        return a

    def setl(self, pose):
        self.settings.setl( pose )
        self.new_world()

    def getl(self):
        return self.world.getl()

    # def move_tool(self, translate):
    #     x, y, z = translate
    #     self.Tool = self.Tool.locate_new( 'Tool', x * self.Tool.i + y * self.Tool.j + z * self.Tool.k )

    def move_to_marker(self):
        self.new_world()
        self.world.move_to_marker()

    def move_tcp(self, translation):
        self.new_world()
        self.world.move_tcp( translation )

    def move_tcp_to_plane(self, offset=0):
        self.new_world()
        self.world.move_tcp_to_plane( offset )

    def set_aruco_marker(self, rvec, tvec):  # rvec: angle axis notation, tvec: coordinates from camera [m]
        # todo: enter tvec and rvec from aruco marker
        # example:
        # rvec = [[-2.6604835],
        #         [-1.18965147],
        #         [0.27949938]]
        #
        # tvec = [[1.887386],
        #         [-10.52237056],
        #         [113.97760565]]

        rvec = np.array( rvec ).flatten()
        tvec = np.array( tvec ).flatten()

        self.settings.aruco_marker_tvec.set( tvec )
        self.settings.aruco_marker_rvec.set( rvec )

    def cuboid_data(self, center, size):
        # suppose axis direction: x: to left; y: to inside; z: to upper
        # get the (left, outside, bottom) point
        o = [a - b / 2 for a, b in zip( center, size )]
        # get the length, width, and height
        l, w, h = size
        x = np.array( [[o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in bottom surface
                       [o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in upper surface
                       [o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in outside surface
                       [o[0], o[0] + l, o[0] + l, o[0], o[0]]] )  # x coordinate of points in inside surface
        y = np.array( [[o[1], o[1], o[1] + w, o[1] + w, o[1]],  # y coordinate of points in bottom surface
                       [o[1], o[1], o[1] + w, o[1] + w, o[1]],  # y coordinate of points in upper surface
                       [o[1], o[1], o[1], o[1], o[1]],  # y coordinate of points in outside surface
                       [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]] )  # y coordinate of points in inside surface
        z = np.array( [[o[2], o[2], o[2], o[2], o[2]],  # z coordinate of points in bottom surface
                       [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],  # z coordinate of points in upper surface
                       [o[2], o[2], o[2] + h, o[2] + h, o[2]],  # z coordinate of points in outside surface
                       [o[2], o[2], o[2] + h, o[2] + h, o[2]]] )  # z coordinate of points in inside surface
        return x, y, z

    @staticmethod
    def get_numpy_mat(vec: sympy.vector.vector.Vector, base):
        a = vec.to_matrix( base )
        # a = DummyVariable.eval_all(a)
        b = dense.matrix2numpy( a, dtype=float )
        return b

    def draw_coordsys(self, coordsys: sympy.vector.coordsysrect.CoordSys3D,
                      text=None, base: sympy.vector.coordsysrect.CoordSys3D = None, **kwargs):
        x_color = 'r'
        y_color = 'g'
        z_color = 'b'
        length = 0.3

        if base is None:
            base = self.world.Robot

        for key, value in kwargs.items():
            if key == "xyz_color":
                x_color, y_color, z_color = value

            if key == 'x_color':
                x_color = value

            if key == 'y_color':
                y_color = value

            if key == 'z_color':
                z_color = value

            if key == 'length':
                length = value
            print( "{0} = {1}".format( key, value ) )

        origin = coordsys.origin.position_wrt( base )
        pos = DigitalWorldHandler.get_numpy_mat( origin, base )
        posx = pos[0][0]
        posy = pos[1][0]
        posz = pos[2][0]

        a = self.get_numpy_mat( coordsys.origin.position_wrt( base ) + length * coordsys.i, base )
        b = self.get_numpy_mat( coordsys.origin.position_wrt( base ) + length * coordsys.j, base )
        c = self.get_numpy_mat( coordsys.origin.position_wrt( base ) + length * coordsys.k, base )

        # Here we create the arrows:

        # a = Arrow3D([0, 1], [0, 0], [0, 0], **self.arrow_prop_dict, color='r')
        # self.ax1.add_artist(a)
        # a = Arrow3D([0, 0], [0, 1], [0, 0], **self.arrow_prop_dict, color='b')
        # self.ax1.add_artist(a)
        # a = Arrow3D([0, 0], [0, 0], [0, 1], **self.arrow_prop_dict, color='g')
        # self.ax1.add_artist(a)

        a = Arrow3D( [posx, a[0][0]], [posy, a[1][0]], [posz, a[2][0]], **self.arrow_prop_dict, color=x_color )
        self.ax1.add_artist( a )
        a = Arrow3D( [posx, b[0][0]], [posy, b[1][0]], [posz, b[2][0]], **self.arrow_prop_dict, color=y_color )
        self.ax1.add_artist( a )
        a = Arrow3D( [posx, c[0][0]], [posy, c[1][0]], [posz, c[2][0]], **self.arrow_prop_dict, color=z_color )
        self.ax1.add_artist( a )

        if text is not None:
            self.ax1.text( posx, posy, posz - 0.01, r'$' + text + '$' )

    def draw_vector(self, v, coordsys, base=None, color='black'):
        if base is None:
            base = self.world.Robot

        origin = coordsys.origin.position_wrt( base )
        pos = DigitalWorldHandler.get_numpy_mat( origin, base )
        posx = pos[0][0]
        posy = pos[1][0]
        posz = pos[2][0]

        # a = self.get_numpy_mat( coordsys.origin.position_wrt( base ) + length * coordsys.i, base )
        # vector = v[0] * coordsys.i + v[1] * coordsys.j + v[2] * coordsys.k

        vec_mat = self.get_numpy_mat( coordsys.origin.position_wrt( base ) + v, base )
        print( 'end' )
        # draw rotation vector at robot base

        a = Arrow3D( [posx, vec_mat[0][0]], [posy, vec_mat[1][0]], [posz, vec_mat[2][0]], **self.arrow_prop_dict,
                     color=color )
        self.ax1.add_artist( a )

    def new_world(self):
        self.world = DigitalWorld( self.settings )

    def plot(self, pause=0):
        if self.plotting:
            self.new_world()
            self.ax1.cla()
            # plot table
            center = [1, 0, -0.150]
            length = 2.000
            width = 0.700
            height = 0.040
            X, Y, Z = self.cuboid_data( center, (length, width, height) )
            self.ax1.plot_surface( X, Y, Z, color='burlywood', rstride=1, cstride=1, alpha=0.1, label="Table", shade=True )

            self.ax1.set_xlabel( 'X' )
            self.ax1.set_xlim( -1, 1 )
            self.ax1.set_ylabel( 'Y' )
            self.ax1.set_ylim( -1, 1 )
            self.ax1.set_zlabel( 'Z' )
            self.ax1.set_zlim( -1, 1 )

            self.draw_coordsys( self.world.Robot, 'Robot' )
            self.draw_coordsys( self.world.Camera, 'Camera' )
            self.draw_coordsys( self.world.Tcp, 'Tcp' )
            # self.draw_coordsys( self.world.UltrasonicTop, 'UTop', xyz_color=('lightsalmon','lightgreen','lightblue'))
            # self.draw_coordsys( self.world.UltrasonicSide, 'USide', xyz_color=('lightsalmon','lightgreen','lightblue'))

            self.draw_coordsys( self.world.ArucoMarker, 'AM' )
            # self.draw_vector( self.world.UltrasonicTopVector, self.world.UltrasonicTop, color='purple')
            # self.draw_vector( self.world.UltrasonicSideVector, self.world.UltrasonicTop, color='purple')

            # Give them a name:
            # self.ax1.text(0.0, 0.0, -0.1, r'$0$')
            self.ax1.text( 1.1, 0, 0, r'$x$' )
            self.ax1.text( 0, 1.1, 0, r'$y$' )
            self.ax1.text( 0, 0, 1.1, r'$z$' )

            # todo: enter robot coordinates and show them
            # position of marker in respect to robot
            plt.ion()
            plt.show()
            if pause:
                plt.pause( pause )

    def plot_tcp(self, pause=0):
        if self.plotting:
            self.new_world()
            self.ax1.cla()
            # plot table
            self.ax1.set_xlabel( 'X' )
            self.ax1.set_xlim( -1, 1 )
            self.ax1.set_ylabel( 'Y' )
            self.ax1.set_ylim( -1, 1 )
            self.ax1.set_zlabel( 'Z' )
            self.ax1.set_zlim( -1, 1 )
            self.draw_coordsys( self.world.Tcp, 'Tcp', base=self.world.Tcp )
            # self.draw_coordsys( self.world.ToolHead, 'ToolHead', base=self.world.Tcp )
            self.draw_coordsys( self.world.UltrasonicTop, 'UltrasonicTop', base=self.world.Tcp )
            self.draw_coordsys( self.world.UltrasonicSide, 'UltrasonicSide', base=self.world.Tcp )

            # self.draw_coordsys( self.world.Camera, 'Camera', base=self.world.Tcp )
            # Give them a name:
            # self.ax1.text(0.0, 0.0, -0.1, r'$0$')
            self.ax1.text( 1.1, 0, 0, r'$x$' )
            self.ax1.text( 0, 1.1, 0, r'$y$' )
            self.ax1.text( 0, 0, 1.1, r'$z$' )

            # todo: enter robot coordinates and show them
            # position of marker in respect to robot
            plt.ion()
            plt.show()
            if pause:
                plt.pause( pause )


class BasicVector( object ):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def set(self, var_list):
        self.x = var_list[0]
        self.y = var_list[1]
        self.z = var_list[2]

    def wrt(self, coordsys: CoordSys3D):
        vec = self.x * coordsys.i + self.y * coordsys.j + self.z * coordsys.k
        if isinstance( vec, sympy.vector.VectorZero ):
            return None
        return vec


class RotationVector( BasicVector ):

    @property
    def angle(self):
        return sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )

    @angle.setter
    def angle(self, value):
        x, y, z = self.axis
        self.x = x * value
        self.y = y * value
        self.z = z * value

    @property
    def axis(self):
        if self.angle:
            return self.x / self.angle, self.y / self.angle, self.z / self.angle
        else:
            return 0.0, 0.0, 0.0

    def axis_wrt(self, coordsys):
        if self.angle:
            vec = self.x / self.angle, self.y / self.angle, self.z / self.angle
        else:
            vec = 0.0, 0.0, 0.0
        x, y, z = vec
        return x * coordsys.i + y * coordsys.j + z * coordsys.k
    # @axis.setter
    # def axis(self, value):
    #     pass


class DigitalWorldSettings( object ):
    def __init__(self):
        self.tcp_tvec = BasicVector()
        self.tcp_rvec = RotationVector()
        self.tool_head_tvec = BasicVector()
        self.tool_head_rvec = RotationVector()

        self.ultrasonic_top_tvec = BasicVector()
        self.ultrasonic_top_rvec = RotationVector()
        self.ultrasonic_top_distance = 0.0
        self.ultrasonic_side_tvec = BasicVector()
        self.ultrasonic_side_rvec_1 = RotationVector()
        self.ultrasonic_side_rvec_2 = RotationVector()
        self.ultrasonic_side_distance = 0.0

        self.camera_tvec = BasicVector()
        self.camera_rvec = RotationVector()

        self.camera_tvec_wrt_tcp = BasicVector()
        self.camera_rvec_wrt_tcp = RotationVector()

        self.aruco_marker_tvec = BasicVector()
        self.aruco_marker_rvec = RotationVector()

        self.cal_points = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0]
        ]

    def setl(self, robot_coord):
        tvec = robot_coord[0:3]
        rvec = robot_coord[3:6]
        self.tcp_tvec.set( tvec )
        self.tcp_rvec.set( rvec )


class DigitalWorld( object ):
    def __init__(self, settings: DigitalWorldSettings):
        self.settings = settings
        self.Robot = CoordSys3D( 'Robot' )

        self.Tcp = self.Robot.orient_new_axis(
            'Tcp',
            settings.tcp_rvec.angle,
            settings.tcp_rvec.axis_wrt( self.Robot ),
            location=settings.tcp_tvec.wrt( self.Robot )
        )

        self.ToolHead = self.Tcp.orient_new_axis(
            'ToolHead',
            settings.tool_head_rvec.angle,
            settings.tool_head_rvec.axis_wrt( self.Tcp ),
            location=settings.tool_head_tvec.wrt( self.Tcp )
        )

        self.UltrasonicTop = self.ToolHead.orient_new_axis(
            'UltrasonicTop',
            settings.ultrasonic_top_rvec.angle,
            settings.ultrasonic_top_rvec.axis_wrt( self.ToolHead ),
            location=settings.ultrasonic_top_tvec.wrt( self.ToolHead )
        )

        self.UltrasonicTopVector = self.UltrasonicTop.k * settings.ultrasonic_top_distance

        self._UltrasonicSide_step_1 = self.ToolHead.orient_new_axis(
            'UltrasonicSide_step_1',
            settings.ultrasonic_side_rvec_1.angle,
            settings.ultrasonic_side_rvec_1.axis_wrt( self.ToolHead )
        )
        self.UltrasonicSide = self._UltrasonicSide_step_1.orient_new_axis(
            'UltrasonicSide',
            settings.ultrasonic_side_rvec_2.angle,
            settings.ultrasonic_side_rvec_2.axis_wrt( self._UltrasonicSide_step_1 ),
            location=settings.ultrasonic_side_tvec.wrt( self.ToolHead )
        )
        self.UltrasonicSideVector = self.UltrasonicSide.k * settings.ultrasonic_side_distance

        self.Camera = self.Robot.orient_new_axis(
            'Camera',
            settings.camera_rvec.angle,
            settings.camera_rvec.axis_wrt( self.Robot ),
            location=settings.camera_tvec.wrt( self.Robot )
        )

        self.ArucoMarker = self.Camera.orient_new_axis(
            'ArucoMarker',
            settings.aruco_marker_rvec.angle,
            settings.aruco_marker_rvec.axis_wrt( self.Camera ),
            location=settings.aruco_marker_tvec.wrt( self.Camera )
        )

        self.set_plane( settings.cal_points )

    def move_to_marker(self, save=True):
        self.Tcp = self.ArucoMarker.orient_new_axis( 'Tcp', sympy.pi, self.ArucoMarker.i )

        if save:
            self.save(
                self.Tcp,
                self.Robot,
                self.settings.tcp_tvec,
                self.settings.tcp_rvec
            )

    def move_tcp(self, translation, save=True):
        if len( translation ) != 3:
            raise Exception( "translation iterable should be of length 3" )
        x, y, z = translation
        vec = x * self.Tcp.i + y * self.Tcp.j + z * self.Tcp.k
        self.Tcp = self.Tcp.locate_new( 'Tcp', vec )

        if save:
            self.save(
                self.Tcp,
                self.Robot,
                self.settings.tcp_tvec,
                self.settings.tcp_rvec
            )

    def move_tcp_to_plane(self, offset=0):
        # make line out of 0,0,0 and point
        b = Point3D(0, 0, 0)
        tcp_point = Point3D( self.getl()[0:3] )
        line = Line3D(b, tcp_point)

        # project line onto plane
        plane_point = self.plane.intersection(line)
        print(plane_point)
        plane_point = plane_point[0]
        print(plane_point)
        # get distance between tcp point and base point
        dis_tcp = b.distance(tcp_point)

        # get distance between plane point and base point
        dis_plane = b.distance(plane_point)

        # put negative or positive sign
        if dis_tcp > dis_plane:
            sign = -1
        else:
            sign = +1
        # get distance to plane

        distance = self.plane.distance( tcp_point ).evalf()
        distance = sign * distance
        # move tcp
        # location = self.plane.projection( tcp_point )
        # print(self.plane.distance( tcp_point ))
        self.move_tcp( (0, 0, distance - offset) )

    def save(self, coordsys, base, tvec, rvec):
        translation_vector = self.get_tvec( coordsys, base )
        rotation_vector = self.get_rvec( coordsys, base )
        tvec.set( translation_vector )
        rvec.set( rotation_vector )

    def get_rvec(self, coordsys, base):
        rot_mat = base.rotation_matrix( coordsys )
        rot_mat_np = dense.matrix2numpy( rot_mat, dtype=float )

        rotation_vector, _ = cv2.Rodrigues( rot_mat_np )
        rotation_vector = rotation_vector.flatten()
        return rotation_vector

    def get_tvec(self, coordsys, base):
        return DigitalWorldHandler.get_numpy_mat( coordsys.origin.position_wrt( base ), base ).flatten()

    def getl(self):
        x, y, z = self.get_tvec( self.Tcp, self.Robot )
        rx, ry, rz = self.get_rvec( self.Tcp, self.Robot )
        return [x, y, z, rx, ry, rz]

    def getl_ultrasonic_top(self, save=True):
        print( self.UltrasonicTopVector )
        a = self.UltrasonicTop.origin.position_wrt( self.Robot ) + self.UltrasonicTopVector
        return a.to_matrix( self.Robot )

    def getl_ultrasonic_side(self, save=True):
        print( self.UltrasonicSideVector )
        a = self.UltrasonicSide.origin.position_wrt( self.Robot ) + self.UltrasonicSideVector
        return a.to_matrix( self.Robot )

    def set_plane(self, point_list):
        self.settings.cal_points = point_list
        point_list[0] = Point3D( point_list[0] )
        point_list[1] = Point3D( point_list[1] )
        point_list[2] = Point3D( point_list[2] )
        self.plane = Plane( point_list[0], point_list[1], point_list[2] )

    def rotate_x_tcp(self, angle, angle_in_degrees=False, save=True):
        self.rotate_tcp(angle, angle_in_degrees, axis='i', save=save)

    def rotate_y_tcp(self, angle, angle_in_degrees=False, save=True):
        self.rotate_tcp(angle, angle_in_degrees, axis='j', save=save)

    def rotate_z_tcp(self, angle, angle_in_degrees=False, save=True):
        self.rotate_tcp(angle, angle_in_degrees, axis='k', save=save)

    def rotate_tcp(self, angle, angle_in_degrees=False, axis='i',  save=True):
        if not (axis == 'i' or axis =='j' or axis =='k'):
            raise Exception("not the right axis (i,j or k) selected: {}".format(axis))

        if angle_in_degrees:
            angle = radians(angle)
        if axis == 'i':
            self.Tcp = self.Tcp.orient_new_axis( 'Tcp', angle, self.Tcp.i )
        elif axis == 'j':
            self.Tcp = self.Tcp.orient_new_axis( 'Tcp', angle, self.Tcp.j )
        elif axis == 'k':
            self.Tcp = self.Tcp.orient_new_axis( 'Tcp', angle, self.Tcp.k )

        if save:
            self.save_tcp()

    def save_tcp(self):
        self.save(
            self.Tcp,
            self.Robot,
            self.settings.tcp_tvec,
            self.settings.tcp_rvec
        )

    def get_angle_xy_plane_tcp(self):
        """
        calculate the angle between the tcp and the xy plane of the Robot
        """
        from sympy import Plane, Point3D, Line3D

        normal_point = Point3D( 0, 0, 1 )
        l_point: Point3D = Point3D( self.Robot.i.to_matrix( self.Tcp ) )

        p: Plane = Plane(
            (0, 0, 0),
            normal_vector=normal_point
        )
        projection = p.projection( l_point )
        param = p.parameter_value( projection, u, v )
        angle = atan2( param[v], param[u] )
        return angle

    def align_tcp_to_robot(self, save=True):
        angle = self.get_angle_xy_plane_tcp()
        self.Tcp = self.Tcp.orient_new_axis( 'Tcp', angle, self.Tcp.k )

        if save:
            self.save_tcp()

    def set_camera_to_tcp(self, save=True):
        self.Camera = self.Tcp.orient_new_axis(
            'Camera',
            self.settings.camera_rvec_wrt_tcp.angle,
            self.settings.camera_rvec_wrt_tcp.axis_wrt( self.Tcp ),
            location=self.settings.camera_tvec_wrt_tcp.wrt( self.Tcp )
        )

        # reset camera back to tvec and rvec
        print(type(self.get_tvec( self.Camera, self.Robot )))
        print(self.get_tvec( self.Camera, self.Robot ))
        print()
        # todo: save
        if save:
            # get tvec and rvec
            self.save(self.Camera, self.Robot,
                      self.settings.camera_tvec, self.settings.camera_rvec)

if __name__ == '__main__':
    world = DigitalWorldHandler()

    # import van_robot
    # import time
    #
    # rob = van_robot.Robot("192.168.0.112", True)  # Set IP address
    # orig_tcp = (0, 0, 0.150, 0, 0, 0)
    # rob.set_tcp(orig_tcp)  # Set Tool Center Point
    # rob.set_payload(0.5, (0, 0, 0))  # Set payload for joints
    # time.sleep(0.2)  # leave some time to robot to process the setup commands
    # a, v = (0.05, 0.5)

    # pos_start = rob.getl()
    # print(pos_start)

    rvec = [-2.98502296, -0.35881683, 0.50341339]
    tvec = [0.31787465, -0.05811111, 1.12995854]

    # world.settings.tcp_tvec.set([0,1,1])
    # world.move_tool( [0, 0, -0.03] )
    world.plot()

    world.set_aruco_marker( rvec, tvec )

    world.move_to_marker()
    world.plot()

    print( 'move to marker' )
    world.move_tcp( (0, 0, -0.2) )
    print( 'moved to new position' )
    world.plot( 2 )
    # world.world.get_angle_xy_plane(world.world.Tcp.i)
    world.world.align_tcp_to_robot()
    world.move_tcp( (0, 0, 0.150) )

    world.set_ultrasonic_side_distance( 0.100 )
    world.plot()
    # pose = world.get_l_aruco_marker()
    # world.plot()
    print( world.getl() )
    # rob.movel(world.getl())
    # pos_start[3:6] = pose[3:6]
    # pos_start[3], pos_start[4], pos_start[5] = rvec
    # print(pos_start)

    plt.pause( 1000 )
