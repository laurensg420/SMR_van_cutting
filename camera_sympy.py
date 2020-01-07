from sympy.vector import CoordSys3D
from sympy.vector import express
from sympy.plotting import *
from sympy import Symbol
import sympy.vector
import sympy
import sympy.abc

theta = Symbol('theta')

# example aruco marker values [cm]
aruco_rvec = [[-3.08916429],
              [-0.09167706],
              [-0.01142135]]
aruco_tvec = [[-12.17256314],
              [-6.86974934],
              [119.01236545]]
aruco_x = aruco_tvec[0][0] / 100
aruco_y = aruco_tvec[2][0] / 100  # y and z-axis are switched in the robot coordinates!!!
aruco_z = aruco_tvec[1][0] / -100  # y and z-axis are switched in the robot coordinates!!!
aruco_angle = 0

theta = aruco_angle
x, y, z = 0.485, -0.295, 0.456  # coordinates of camera in respect to robot coord [m]
a, b, c = aruco_x, aruco_y, aruco_z

Robot = CoordSys3D('Robot')
Cam = Robot.locate_new('Cam', x * Robot.i + y * Robot.j + z * Robot.k)
Cam2 = Cam.orient_new_axis('Cam2', theta, Cam.i)
Marker = Cam2.locate_new('Marker', a * Cam2.i + b * Cam2.j + c * Cam2.k)

mat = Marker.position_wrt(Robot).to_matrix(Robot)
# mat = Robot.position_wrt(Marker).to_matrix(Marker)

from sympy.matrices import dense

mat = dense.matrix2numpy(mat, float)
print(mat)
MarkerTurned = Marker.orient_new_axis('MarkerTurned', aruco_angle, Marker.k)
#
# print(Cam.position_wrt(Robot))
# print(Robot.origin.express_coordinates(Cam2))
# print(Cam.origin.express_coordinates(Robot))
# print(Robot.origin.express_coordinates(Marker))
# print(Marker.origin.express_coordinates(Robot))
# print(MarkerTurned.rotation_matrix(Robot))
# print(express(MarkerTurned.position_wrt(Robot), MarkerTurned))


from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib as plt
import random

fig = pyplot.figure()
ax = Axes3D(fig)
ax._hold

ax.scatter(0, 0, 0, label='Robot position')
ax.scatter(x, y, z, label='Camera position')
ax.scatter(mat[0][0], mat[1][0], mat[2][0], label='Label position')

ax.legend()
ax.set_xlabel("x-axis")
ax.set_ylabel("y-axis")
ax.set_zlabel("z-axis")

yy, xx = np.meshgrid(range(2), range(2))

xx = xx - 0.09
yy = yy - 0.35
zz = yy * 0 - 0.15
ax.plot_surface(xx, yy, zz, color='burlywood', label="Table")

pyplot.show()

# import numpy as np
#
# v_camera = [0,0.5,0.2]
# v_camera_aruko = [0.5,0.2,0.6]
# origin = [0,0,0]
# sum = np.add(v_camera, v_camera_aruko)
#
# #sum = v_camera.row(0).dot(v_camera_aruko)
#
# #sympy.DeferredVector
# #Dist_0 = v_camera + v_camera_aruko
#
# print(sum)
# print(length)
