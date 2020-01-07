# Robot imports
import van_robot
import cv2
import time
# Create origin
import create_origin

# Aruco imports
from cv2 import aruco
import yaml
from pathlib import Path
from sympy.vector import CoordSys3D
from matplotlib import pyplot

# Calibration imports
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from sympy import *
from sympy import Symbol
import arduino_serial
from tqdm import tqdm

# root directory of repo for relative path specification.
root = Path(__file__).parent.absolute()

# board properties

# aruco example
board_params = {
    "board_type": "aruco",
    "dict_type": aruco.DICT_6X6_1000,
    "markersX": 4,
    "markersY": 5,
    "markerLength": 3.75,  # Provide length of the marker's side [cm]
    "markerSeparation": 0.5,  # Provide separation between markers [cm]
}
# For validating results, show aruco board to camera.
board_params["aruco_dict"] = aruco.getPredefinedDictionary(board_params["dict_type"])

"""# charuco example
board_params_params = {
    "board_type": "charuco",
    "dict_type": aruco.DICT_4X4_1000,
    "squaresX": 4,
    "squaresY": 4,
    "squareLength": 3,  # Provide length of the square's side [cm]
    "markerLength": 3 * 7 / 9,  # Provide length of the marker's side [cm]
}
# For validating results, show aruco board to camera.
board_params["aruco_dict"] = aruco.getPredefinedDictionary(board_params["dict_type"])
"""

# create arUco board
if board_params["board_type"] == "aruco":
    board = aruco.GridBoard_create(board_params["markersX"], board_params["markersY"], board_params["markerLength"],
                                   board_params["markerSeparation"], board_params["aruco_dict"])
elif board_params["board_type"] == "charuco":
    board = aruco.CharucoBoard_create(board_params["squaresX"], board_params["squaresY"], board_params["squareLength"],
                                      board_params["markerLength"], board_params["aruco_dict"])
else:
    raise ValueError("Couldn't find correct board type (aruco or charuco)")

"""uncomment following block to draw and show the board"""
# img = board.draw((864, 1080))
# cv2.imshow("aruco", img)
# cv2.imwrite("aruco.png", img)

arucoParams = aruco.DetectorParameters_create()

# Set second camera outside laptop
# cv2.CAP_DSHOW removes delay on logitech C920 webcam
camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
# Turn the autofocus off
camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)

# Open calibration file created with data_generation.py
with open('aruco/calibration.yaml') as f:
    loadeddict = yaml.safe_load(f)
mtx = loadeddict.get('camera_matrix')
dist = loadeddict.get('dist_coeff')
mtx = np.array(mtx)
dist = np.array(dist)

ret, img = camera.read()
img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
h, w = img_gray.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

pose_r, pose_t = [], []

while True:
    ret, img = camera.read()
    img_aruco = img
    im_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    h, w = im_gray.shape[:2]
    dst = cv2.undistort(im_gray, mtx, dist, None, newcameramtx)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, board_params["aruco_dict"], parameters=arucoParams)
    # cv2.imshow("original", img_gray)
    if corners == None:
        raise Exception("No aruco corners found!")
    else:
        rvec = None
        tvec = None
        ret, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, newcameramtx, dist, rvec,
                                                  tvec)  # For a board

        print("Rotation ", rvec, "\nTranslation", tvec, "\n=======")
        if ret != 0:
            img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0, 255, 0))
            img_aruco = aruco.drawAxis(img_aruco, newcameramtx, dist, rvec, tvec,
                                       10)  # axis length 100 can be changed according to your requirement
        cv2.imshow("World co-ordinate frame axes", img_aruco)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
cv2.destroyAllWindows()

# output from aruco
theta = Symbol('theta')

# aruco marker values [cm]
aruco_rvec = [rvec[0][0], rvec[1][0], rvec[2][0]]
aruco_tvec = [tvec[0][0], tvec[1][0], tvec[2][0]]

# x-value is off by 84 mm
# y-value is off by 4 mm
# z-value is off by 25 mm
aruco_x = aruco_tvec[0] / 100
aruco_y = aruco_tvec[2] / 100  # y and z-axis are switched in the robot coordinates!!!
aruco_z = aruco_tvec[
              1] / -100  # y and z-axis are switched in the robot coordinates!!! the minus is to compensate for axis orientation
aruco_angle = 0

theta = aruco_angle
x, y, z = 0.485, -0.295, 0.456  # coordinates of camera in respect to robot coord [m]
a, b, c = aruco_x, aruco_y, aruco_z  # coordinates of marker in respect to the camera [m]

# translate 3d coordinate systems
Robot = CoordSys3D('Robot')  # base coordinate system
Cam = Robot.locate_new('Cam', x * Robot.i + y * Robot.j + z * Robot.k)
Cam2 = Cam.orient_new_axis('Cam2', theta, Cam.i)
Marker = Cam2.locate_new('Marker', a * Cam2.i + b * Cam2.j + c * Cam2.k)

# Calculate marker position with respect to the robots base
mat = Marker.position_wrt(Robot).to_matrix(Robot)
mat = dense.matrix2numpy(mat, float)

fig = pyplot.figure()
ax = Axes3D(fig)

ax.scatter(0, 0, 0, label='Robot position')
ax.scatter(x, y, z, label='Camera position')

ax.scatter(mat[0][0], mat[1][0], mat[2][0], label='Label position')

ax.legend()
ax.set_xlabel("x-axis")
ax.set_ylabel("y-axis")
ax.set_zlabel("z-axis")

# Plot positions of robot_base, camera and position of the marker in 3D
# todo turn on plot for info (turned off for fast testing)
# pyplot.show()

# Robot movements
# todo: Moving x, y and z-plane robot
# todo: Line up with aruco Marker (0.2m y-offset from board)

rob = van_robot.Robot("192.168.0.1", True)  # Set IP address
orig_tcp = (0, 0, 0.150, 0, 0, 0)
rob.set_tcp(orig_tcp)  # Set Tool Center Point
rob.set_payload(0.5, (0, 0, 0))  # Set payload for joints
time.sleep(0.2)  # leave some time to robot to process the setup commands
a, v = (0.05, 0.5)  # acceleration and movementspeed of robot

# Start from the pose the robot is in
pos0 = rob.getl()

# Tool orientation [rx, ry, rz]
tool_orient = [0, -2.221441469, -2.221441469]

# Put the X, Y, Z of start pose in (last 3 coordinates are tool head positioning)
pos_start = [pos0[0], pos0[1], pos0[2], tool_orient[0], tool_orient[1], tool_orient[2]]

# Take the Y-distance 0.10[m] out of the van
safe_dist = mat[1] - 0.12

# Move to the aruco marker point
# todo turn on move command
rob.movel((mat[0], safe_dist, mat[2], tool_orient[0], tool_orient[1], tool_orient[2]), a, v)

# Calibration Y-plane and X,Z-Axis

# Declare the port and baudrate for the Arduino and connect to the Arduino
# todo CHECK COM!
arduino = arduino_serial.Arduino("COM3")

# start position of the robot (from aruco marker)
# assumed is a robot start position which is close to the centre of the measured plane
pos_start = (mat[0], safe_dist, mat[2], tool_orient[0], tool_orient[1], tool_orient[2])

# Take safe offset from Y-plan after global aruco Y-value
aruco_offset = arduino.get_multiple_average(arduino.get_distance_top, 100) / 1000  # [m]
aruco_offset -= 0.008  # correct for 8 mm of tcp offset of ultrasonic sensor
triangle_safe_dis = aruco_offset - 0.10
rob.translate((0, triangle_safe_dis, 0, 0, 0, 0), a, v)
round(triangle_safe_dis, -3)
print("Robot is gonna move", triangle_safe_dis, "[m] towards the van.")

# Define Y-plane
rob.measure_triangle(arduino, 0.100, a, v)

# Define Z,X-axis
# rob.side_look(arduino)
rob.move_y_plane(-0.040)

points1 = rob.measure_sides(arduino)
cur_pos = rob.getl()
side_margin = 0.100  # [m]
movement = 0.100  # [m]
side, distance, point_pos = points1[0]
if distance > (side_margin + movement):
    # move to left / negative x
    cur_pos[0] -= movement
else:
    cur_pos[0] += movement
# move to right / positive x

side, distance, point_pos = points1[1]
if distance > (side_margin + movement):
    # move to top / positive z
    cur_pos[2] += movement
else:
    # move to bottom / negative z
    cur_pos[2] -= movement

rob.move_y_plane(-0.030, cur_pos)
#
print("Current tool pose is: ", rob.getl())

points2 = rob.measure_sides(arduino)

# Create origin

# coordinates in [X,Z]

# SIDE Coordinates [z-formula]
coor1 = [points1[0][2][0], points1[0][2][2]]  # [X,Z]
coor2 = [points2[0][2][0], points2[0][2][2]]

# TOP Coordinates [x-formula]
coor3 = [points1[1][2][0], points1[1][2][2]]
coor4 = [points2[1][2][0], points2[1][2][2]]

# create_origin.formula_line('x',coor1,coor2)
# create_origin.formula_line('z',coor3,coor4)
line_x = create_origin.formula_line('x', coor1, coor2)
line_z = create_origin.formula_line('z', coor3, coor4)
origin = create_origin.intersect_lines(line_x, line_z)
print(origin)  # [X,Z]

# Calculate translation to origin
cur_pos = rob.getl()
cur_pos[0] = origin[0]
cur_pos[2] = origin[1]
# Move to origin (first moves out, then in x,z-plane)
rob.move_y_plane(-0.15, cur_pos)
