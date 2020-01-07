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

# root directory of repo for relative path specification.
root = Path( __file__ ).parent.absolute()

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
board_params["aruco_dict"] = aruco.getPredefinedDictionary( board_params["dict_type"] )

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
    board = aruco.GridBoard_create( board_params["markersX"], board_params["markersY"], board_params["markerLength"],
                                    board_params["markerSeparation"], board_params["aruco_dict"] )
elif board_params["board_type"] == "charuco":
    board = aruco.CharucoBoard_create( board_params["squaresX"], board_params["squaresY"], board_params["squareLength"],
                                       board_params["markerLength"], board_params["aruco_dict"] )
else:
    raise ValueError( "Couldn't find correct board type (aruco or charuco)" )

"""uncomment following block to draw and show the board"""
# img = board.draw((864, 1080))
# cv2.imshow("aruco", img)
# cv2.imwrite("aruco.png", img)

arucoParams = aruco.DetectorParameters_create()


def list_cameras():
    camera = cv2.VideoCapture()
    device_counts = 0
    device_imgs = []
    while True:
        if not camera.open(device_counts, cv2.CAP_DSHOW ):
            break
        _, img = camera.read()
        device_imgs.append(img)
        device_counts += 1

    camera.release()
    return device_counts, device_imgs



# Set second camera outside laptop
# cv2.CAP_DSHOW removes delay on logitech C920 webcam
def connect(camera_index):
    global camera, mtx, dist, h, w, newcameramtx, roi, pose_r, pose_t
    camera = cv2.VideoCapture( camera_index, cv2.CAP_DSHOW )
    print(camera)
    camera.set( cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc( 'M', 'J', 'P', 'G' ) )
    camera.set( cv2.CAP_PROP_FRAME_WIDTH, 960 )
    camera.set( cv2.CAP_PROP_FRAME_HEIGHT, 540 )
    # Turn the autofocus off
    camera.set( cv2.CAP_PROP_AUTOFOCUS, 0 )

    if camera is None or not camera.isOpened():
        raise Exception('Warning: unable to open video source')

    # Open calibration file created with data_generation.py
    with open( 'aruco/calibration.yaml' ) as f:
        loadeddict = yaml.safe_load( f )
    mtx = loadeddict.get( 'camera_matrix' )
    dist = loadeddict.get( 'dist_coeff' )
    mtx = np.array( mtx )
    dist = np.array( dist )

    ret, img = camera.read()

    img_gray = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY )
    h, w = img_gray.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix( mtx, dist, (w, h), 1, (w, h) )

    pose_r, pose_t = [], []


def detect_marker(imshow: bool = True):
    global camera, mtx, dist, h, w, newcameramtx, roi, pose_r, pose_t
    ret, img = camera.read()
    ret, img = camera.read()
    img_aruco = img
    im_gray = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY )
    h, w = im_gray.shape[:2]
    dst = cv2.undistort( im_gray, mtx, dist, None, newcameramtx )
    corners, ids, rejectedImgPoints = aruco.detectMarkers( dst, board_params["aruco_dict"], parameters=arucoParams )
    # cv2.imshow("original", img_gray)
    if not len( corners ):
        # raise Exception( "No aruco corners found!" )
        print( 'No corners' )
        if imshow:
            cv2.imshow( "World co-ordinate frame axes", img )
        return None, None
    else:
        rvec = None
        tvec = None
        ret, rvec, tvec = aruco.estimatePoseBoard( corners, ids, board, newcameramtx, dist, rvec,
                                                   tvec )  # For a board

        print( "Rotation ", rvec, "\nTranslation", tvec, "\n=======" )
        if ret != 0:
            img_aruco = aruco.drawDetectedMarkers( img, corners, ids, (0, 255, 0) )
            img_aruco = aruco.drawAxis( img_aruco, newcameramtx, dist, rvec, tvec,
                                        10 )  # axis length 100 can be changed according to your requirement
        if imshow:
            cv2.imshow( "World co-ordinate frame axes", img_aruco )

        rvec = np.array( rvec ).flatten()
        tvec = np.array( tvec ).flatten()
        tvec = tvec / 100  # from [cm] to [m]
        return rvec, tvec


# x-value is off by 84 mm
# y-value is off by 4 mm
# z-value is off by 25 mm
x, y, z = 0.485, -0.295, 0.456  # coordinates of camera in respect to robot coord [m]

if __name__ == "__main__":
    connect()
    while True:
        rvec, tvec = detect_marker( imshow=True )
        print( "{}, {}".format( rvec, tvec ) )
        if cv2.waitKey( 0 ) & 0xFF == ord( 'q' ):
            break
    cv2.destroyAllWindows()
