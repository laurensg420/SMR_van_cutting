"""
This code assumes that images used for calibration are of the same arUco marker board provided with code

"""

import cv2
from cv2 import aruco
import yaml
import numpy as np
from pathlib import Path
from tqdm import tqdm

# root directory of repo for relative path specification.
root = Path(__file__).parent.absolute()

# Set this flsg True for calibrating camera and False for validating results real time
calibrate_camera = False

# Set path to the images
calib_imgs_path = root.joinpath("aruco_data") # Changed!

# board props
# aruco example
board_params = {
    "board_type": "aruco",
    "dict_type": aruco.DICT_6X6_1000,
    "markersX": 4,
    "markersY": 5,
    "markerLength": 3.75,  # Provide length of the marker's side [cm]
    "markerSeparation": 0.5,  # Provide separation between markers [cm]
}
# # For validating results, show aruco board to camera.
# board_params["aruco_dict"] = aruco.getPredefinedDictionary(board_params["dict_type"])

# board_params = {
#     "board_type": "aruco",
#     "dict_type": aruco.DICT_6X6_1000,
#     "markersX": 2,
#     "markersY": 2,
#     "markerLength": 3.75,  # Provide length of the marker's side [cm]
#     "markerSeparation": 0.5,  # Provide separation between markers [cm]
# }
# For validating results, show aruco board to camera.
board_params["aruco_dict"] = aruco.getPredefinedDictionary(board_params["dict_type"])
# charuco example
# board_params_params = {
#     "board_type": "charuco",
#     "dict_type": aruco.DICT_4X4_1000,
#     "squaresX": 4,
#     "squaresY": 4,
#     "squareLength": 3,  # Provide length of the square's side [cm]
#     "markerLength": 3 * 7 / 9,  # Provide length of the marker's side [cm]
# }
# # For validating results, show aruco board to camera.
# board_params["aruco_dict"] = aruco.getPredefinedDictionary(board_params["dict_type"])


# create arUco board
if board_params["board_type"] == "aruco":
    board = aruco.GridBoard_create(board_params["markersX"], board_params["markersY"], board_params["markerLength"],
                                   board_params["markerSeparation"], board_params["aruco_dict"])
elif board_params["board_type"] == "charuco":
    board = aruco.CharucoBoard_create(board_params["squaresX"], board_params["squaresY"], board_params["squareLength"],
                                      board_params["markerLength"], board_params["aruco_dict"])
else:
    raise ValueError("Couldn't find correct board type (aruco or charuco)")

'''uncomment following block to draw and show the board'''
# img = board.draw((864, 1080))
# cv2.imshow("aruco", img)
# cv2.imwrite("aruco.png", img)

arucoParams = aruco.DetectorParameters_create()

if calibrate_camera == True:
    img_list = []
    calib_fnms = calib_imgs_path.glob('*.jpg')
    print('Using ...', end='')
    for idx, fn in enumerate(calib_fnms):
        print(idx, '', end='')
        img = cv2.imread(str(root.joinpath(fn)))
        img_list.append(img)
        h, w, c = img.shape
    print('Calibration images')

    counter, corners_list, id_list = [], [], []
    first = True
    for im in tqdm(img_list):
        img_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, board_params["aruco_dict"], parameters=arucoParams)

        if first == True:
            corners_list = corners
            id_list = ids
            first = False
        else:
            corners_list = np.vstack((corners_list, corners))
            id_list = np.vstack((id_list, ids))
        counter.append(len(ids))
    print('Found {} unique markers'.format(np.unique(ids)))

    counter = np.array(counter)
    print("Calibrating camera .... Please wait...")
    # mat = np.zeros((3,3), float)
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(corners_list, id_list, counter, board, img_gray.shape,
                                                              None, None)

    print("Camera matrix is \n", mtx,
          "\n And is stored in calibration.yaml file along with distortion coefficients : \n", dist)
    data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
    with open("calibration.yaml", "w") as f:
        yaml.dump(data, f)

else:
    camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    # TODO: Remember to put size!
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # turn the autofocus off

    ret, img = camera.read()

    with open('calibration.yaml') as f:
        loadeddict = yaml.safe_load(f)
    mtx = loadeddict.get('camera_matrix')
    dist = loadeddict.get('dist_coeff')
    mtx = np.array(mtx)
    dist = np.array(dist)

    ret, img = camera.read()
    # cv2.imshow("imgw", img)
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
            print("pass")
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

            if cv2.waitKey(0) & 0xFF == ord('q'):
                break
        cv2.imshow("World co-ordinate frame axes", img_aruco)

cv2.destroyAllWindows()
