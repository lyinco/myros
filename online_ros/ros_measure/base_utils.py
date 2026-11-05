import cv2
import numpy as np
# expected_dist_mm = 159.45
# tar_idx = [2, 87]

def load_calib(calib_file = r'..\online_calib\src\online_calib\test.json'):

    with open(calib_file) as f:
        calib_data = json.load(f)

    m1 = np.array(calib_data["m1"])
    m2 = np.array(calib_data["m2"])
    d1 = np.array(calib_data["d1"])
    d2 = np.array(calib_data["d2"])
    R = np.array(calib_data["R"])
    T = np.array(calib_data["T"])
    ret = calib_data["ret"]

    return m1, m2, d1, d2, R, T


def get_charuco_board(squares_x =12, squares_y =9, square_length =0.015, marker_length =0.01175, dict_type = cv2.aruco.DICT_5X5_1000):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
    if hasattr(cv2.aruco, 'CharucoBoard'):
        board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length,
            marker_length,
            dictionary
        )
    else:
        # aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        board = cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, dictionary)
    return board, dictionary


def find_charuco_corners(gray, board, win=(5, 5), max_iter=50, eps=1e-4):
    if gray is None:
        return None, None

    if board is None:
        board, aruco_dict  = get_charuco_board()

    if hasattr(cv2.aruco, 'CharucoDetector'):
        detector = cv2.aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is None or len(ids) == 0:
            return None, None
        retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board)

    if charuco_ids is None or len(charuco_ids) == 0:
        return None, None

    corners = charuco_corners.reshape(-1, 1, 2).astype(np.float32)
    cv2.cornerSubPix(
        gray, corners, win, (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iter, eps))
    return corners.reshape(-1, 1, 2), charuco_ids



