import os.path

import cv2
from typing import Optional, List, Tuple
from pathlib import Path
import numpy as np
from glob import glob


class CalibrationResult:
    def __init__(self):
        self.ret: Optional[float] = 0.0
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        # self.rvecs: List[np.ndarray] = []
        # self.tvecs: List[np.ndarray] = []
        self.image_details: List[CalibImage] = []


class CalibrationResultStereo:
    def __init__(self):
        self.ret: Optional[float] = 0.0
        self.R: Optional[np.ndarray] = None
        self.T: Optional[np.ndarray] = None
        self.E: Optional[np.ndarray] = None
        self.F: Optional[np.ndarray] = None


class CalibImage:
    def __init__(self, path: str):
        self.path = path
        self.name = Path(path).name
        self.thumbnail = None
        self.reproj_error: Optional[float] = None
        self.rms_error: Optional[float] = None
        self.corners_found: bool = False

        self.image_size: Optional[np.ndarray] = None
        self.corners_points: Optional[np.ndarray] = None
        self.corners_ids: Optional[np.ndarray] = None
        self.object_points: List[np.ndarray] = None
        self.image_points: List[np.ndarray] = None
        self.reproj_points: List[np.ndarray] = None
        self.reproj_errors: List[float] = None

        self.rvec: np.ndarray = None
        self.tvec: np.ndarray = None


def find_charuco_corners(gray, board, win=(5, 5), max_iter=50, eps=1e-4):
    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
    if charuco_ids is None or len(charuco_ids) == 0:
        return None, None

    corners = charuco_corners.reshape(-1, 1, 2).astype(np.float32)
    cv2.cornerSubPix(
        gray, corners, win, (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iter, eps))
    return corners.reshape(-1, 1, 2), charuco_ids



def find_charuco_corners_new(gray, board, win=(5, 5), max_iter=50, eps=1e-4):
    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
    corners = charuco_corners.reshape(-1, 1, 2).astype(np.float32)
    cv2.cornerSubPix(
        gray, corners,
        winSize=win,  # (5,5) 窗口
        zeroZone=(-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                  max_iter, eps))
    if charuco_ids is not None and len(charuco_ids) > 0:
        return charuco_corners, charuco_ids
    return None, None



def find_charuco_markers(gray, dict_type = cv2.aruco.DICT_5X5_1000):
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(gray)
    if ids is not None:
        ids = ids.flatten()
        return corners, ids
    return None, None


def get_charuco_board(squares_x =12, squares_y =9, square_length =0.015, marker_length =0.01175, dict_type = cv2.aruco.DICT_5X5_1000):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length,
        marker_length,
        dictionary
    )
    return board


def view_board():
    board = get_charuco_board()
    pixels_per_square = 100

    img = board.generateImage(
        outSize=(board.getChessboardSize()[0] * pixels_per_square,
                 board.getChessboardSize()[1] * pixels_per_square),
        marginSize=20,  # 白边像素
        borderBits=1
    )
    cv2.imshow("charuco", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def view_corners(gray, board):
    # 转成彩色图方便画彩色
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    charuco_corners, charuco_ids = find_charuco_corners(gray, board)
    if charuco_ids is None:
        print("未检测到 ChArUco 角点")
        return vis

    # 2) 画出 ChArUco 角点 + ID
    cv2.aruco.drawDetectedCornersCharuco(
        vis,
        charuco_corners,
        charuco_ids,
        cornerColor=(0, 0, 255)  # 红色角点
    )

    print(f"检测到 {len(charuco_ids)} 个 ChArUco 角点")
    return vis

def detect_images_corners(images_set: List[CalibImage], board):
    all_corners = []
    all_ids = []
    valid_images_path = []

    image_size = None
    for img in images_set:
        gray = cv2.imread(img.path, cv2.IMREAD_GRAYSCALE)
        if image_size is None:
            image_size = gray.shape[::-1]
        charuco_corners, charuco_ids = find_charuco_corners(gray, board)
        if charuco_corners is not None and charuco_ids is not None:
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)
            valid_images_path.append(img.path)

            img.corners_found = True
            img.image_size = image_size
            img.corners_points = charuco_corners
            img.corners_ids = charuco_ids

    return all_corners, all_ids, valid_images_path, image_size, images_set


def calibrate_charuco(image_set: List[CalibImage], squares_x=12, squares_y=9, square_length=0.015,
                      marker_length=0.01175, dict_type=cv2.aruco.DICT_5X5_1000):

    board = get_charuco_board(squares_x, squares_y, square_length, marker_length, dict_type)
    all_charuco_corners, all_charuco_ids, valid_images_path, image_size, images_set = detect_images_corners(
        image_set, board)

    object_points = []
    image_points = []
    valid_idx = []
    for ii, img in enumerate(images_set):
        if not img.corners_found:
            continue
        valid_idx.append(ii)
        obj_points, img_points = board.matchImagePoints(img.corners_points, img.corners_ids)
        if obj_points is not None and img_points is not None:
            img.object_points = obj_points
            img.image_points = img_points
            object_points.append(obj_points)
            image_points.append(img_points)

    camera_matrix = np.eye(3, dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, camera_matrix, dist_coeffs,
        # flags=cv2.CALIB_FIX_PRINCIPAL_POINT
        flags=0
    )

    mean_error, reprojection_points, reprojection_errors = calc_reprojection_points(
        object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs
    )

    print("calibration result: ")
    print(f'mean error = {mean_error}')
    print(f'num reprojection_points = {len(reprojection_points)}')
    for ii, _ in enumerate(reprojection_points):
        idx = valid_idx[ii]
        image_set[idx].reproj_points = reprojection_points[ii]
        image_set[idx].reproj_error = reprojection_errors[ii]
        image_set[idx].reproj_errors = np.linalg.norm(
            reprojection_points[ii].reshape(-1, 2) - image_points[ii].reshape(-1, 2), axis=1)
        image_set[idx].rvec = rvecs[ii]
        image_set[idx].tvec = tvecs[ii]

    calib_res = CalibrationResult()
    calib_res.ret = ret
    calib_res.camera_matrix = camera_matrix
    calib_res.dist_coeffs = dist_coeffs
    calib_res.mean_error = mean_error
    calib_res.image_details = image_set
    return calib_res


def calibrate_charuco_stereo(calib_left, calib_right):

    mm1 = calib_left.camera_matrix
    dd1 = calib_left.dist_coeffs
    mm2 = calib_right.camera_matrix
    dd2 = calib_right.dist_coeffs
    image_size = calib_left.image_details[0].image_size

    pts_lefts = []
    pts_rights = []
    pts_objs = []
    for ii, details in enumerate(calib_left.image_details):
        ldata = details
        rdata = calib_right.image_details[ii]
        idx_left = ldata.corners_ids.reshape(-1).tolist()
        idx_right = rdata.corners_ids.reshape(-1).tolist()
        common_idx = list(set(idx_left).intersection(set(idx_right)))
        common_idx.sort()
        pts_left, pts_right, objpoints = [], [], []
        for cidx in common_idx:
            idxl = idx_left.index(cidx)
            idxr = idx_right.index(cidx)
            pts_left.append(ldata.corners_points[idxl].reshape(-1))
            objpoints.append(ldata.object_points[idxl].reshape(-1))
            pts_right.append(rdata.corners_points[idxr].reshape(-1))

        pts_lefts.append(np.array(pts_left, dtype=np.float32))
        pts_rights.append(np.array(pts_right, dtype=np.float32))
        pts_objs.append(np.array(objpoints, dtype=np.float32))

    print('start run core stereo calibrate')
    retval, mm1, dd1, mm2, dd2, R, T, E, F = cv2.stereoCalibrate(pts_objs, pts_lefts, pts_rights,
                                                                 mm1, dd1, mm2, dd2,
                                                                 image_size,
                                                                 R=None, T=None,
                                                                 E=None, F=None,
                                                                 flags=0,
                                                                 criteria=(
                                                                 cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                                                                 30, 1e-6))
    calib_stereo_res = CalibrationResultStereo()
    calib_stereo_res.ret = retval
    calib_stereo_res.R = R
    calib_stereo_res.T = T
    calib_stereo_res.E = E
    calib_stereo_res.F = F
    print('stereo done. ')
    print(f'R= {R.tolist()}')
    return calib_stereo_res


def calc_reprojection_points(object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs):

    total_points = 0
    total_error = 0
    reprojection_errors = []
    reprojection_points = []

    print(f'images num: {len(object_points)}')
    for i in range(len(object_points)):
        imgpoints2, _ = cv2.projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(image_points[i], imgpoints2, cv2.NORM_L2)
        total_error += error ** 2
        total_points += len(object_points[i])
        reprojection_points.append(imgpoints2)
        reprojection_errors.append(error / np.sqrt(len(object_points[i])) if len(object_points[i]) > 0 else 0)
    mean_error = np.sqrt(total_error / total_points) if total_points > 0 else 0
    return mean_error, reprojection_points, reprojection_errors


def main_calib_stereo(image_dir = r'F:\git_src\usb_grab\images\0910_105651'):

    files = glob((os.path.join(image_dir, 'left_*.png')))
    files = [os.path.basename(f) for f in files]
    fidx = [int(f.split('_')[-1].replace('.png', '')) for f in files]
    fidx = sorted(fidx)

    left_images = [CalibImage(img_path) for img_path in
                    [os.path.join(image_dir, f"left_{i}.png") for i in fidx]]
    left_res = calibrate_charuco(left_images)
    right_images = [CalibImage(img_path) for img_path in
                    [os.path.join(image_dir, f"right_{i}.png") for i in fidx]]
    right_res = calibrate_charuco(right_images)

    stereo_res = calibrate_charuco_stereo(left_res, right_res)
    print('left result: ')
    print('m = ', left_res.camera_matrix)
    print('d = ', left_res.dist_coeffs)
    print('right result:')
    print('m = ', right_res.camera_matrix)
    print('d = ', right_res.dist_coeffs)
    print('R = ', stereo_res.R)
    print('T = ', stereo_res.T)
    print('ret = ', stereo_res.ret)
    print('baseline = ', np.linalg.norm(stereo_res.T))
    return left_res.camera_matrix, left_res.dist_coeffs, right_res.camera_matrix, right_res.dist_coeffs, stereo_res.R, stereo_res.T, stereo_res.ret


def save_calib(m1, d1, m2, d2, R, T, ret, file_name):
    import pickle
    data = {
        "m1": m1, "d1": d1,
        "m2": m2, "d2": d2,
        "R": R, "T": T,
        "ret": ret,
    }
    with open(file_name, "wb") as f:
        pickle.dump(data, f)


    import json
    data = {
        "m1": m1.tolist(), "d1": d1.tolist(),
        "m2": m2.tolist(), "d2": d2.tolist(),
        "R": R.tolist(), "T": T.tolist(),
        "ret": ret,
    }
    with open(file_name.replace('.pkl', '.json'), "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def charuco_physical_distance(board, id1, id2):
    """
    计算棋盘上两个 ChArUco 角点(十字点)的物理距离(米)
    id 按行优先编号: id = row * (squares_x - 1) + col
    """
    squares_x, squares_y = board.getChessboardSize()
    cols = squares_x - 1           # 内部角点列数
    rows = squares_y - 1           # 内部角点行数

    r1, c1 = divmod(id1, cols)
    r2, c2 = divmod(id2, cols)

    # 物理坐标（单位: 米），原点在第一个内部角点
    p1 = np.array([c1, r1]) * board.getSquareLength()
    p2 = np.array([c2, r2]) * board.getSquareLength()
    return round(np.linalg.norm(p1 - p2), 5)


def test_measure(save_name = 'test.pkl'):

    image_dir = r'E:\shares\git\online_win\images\1015_133730'
    calib_res = main_calib_stereo(image_dir)
    m1, d1, m2, d2, R, T, ret = calib_res
    save_calib(m1, d1, m2, d2, R, T, ret, save_name)

    image_idx = 2
    left_file=os.path.join(image_dir, f'left_{image_idx}.png')
    right_file = os.path.join(image_dir, f'right_{image_idx}.png')
    imagel = cv2.imread(left_file,0)
    imager = cv2.imread(right_file,0)
    board = get_charuco_board()

    charuco_corners1, charuco_ids1 = find_charuco_corners(imagel, board)
    charuco_corners2, charuco_ids2 = find_charuco_corners(imager, board)
    shared_idx = list(set(charuco_ids1.flatten()).intersection(set(charuco_ids2.flatten())))
    shared_idx.sort()
    tar1, tar2 = shared_idx[0], shared_idx[-1]
    expected_dist = charuco_physical_distance(board, tar1, tar2)
    print(expected_dist)

    map1 = dict(zip(charuco_ids1.flatten(), charuco_corners1))
    map2 = dict(zip(charuco_ids2.flatten(), charuco_corners2))


    # 构造投影矩阵
    P1 = m1 @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = m2 @ np.hstack([R, T.reshape(3, 1)])

    # def triangulate(px1, px2):
    #     p1 = np.array([[px1[0][0]], [px1[0][1]]], dtype=np.float64)
    #     p2 = np.array([[px2[0][0]], [px2[0][1]]], dtype=np.float64)
    #     X = cv2.triangulatePoints(P1, P2, p1, p2)  # 4x1
    #     X = X[:3] / X[3]  # 齐次 -> 欧氏
    #     return X.flatten()

    def triangulate(px1, px2):

        p1 = np.array(px1, dtype=np.float64).reshape(1, 1, 2)
        p2 = np.array(px2, dtype=np.float64).reshape(1, 1, 2)

        # 去畸变 -> 归一化平面坐标 (x/z, y/z)
        n1 = cv2.undistortPoints(p1, m1, d1)  # 1x1x2
        n2 = cv2.undistortPoints(p2, m2, d2)

        # 用归一化坐标构造投影矩阵（去掉内参）
        P1n = np.hstack([np.eye(3), np.zeros((3, 1))])
        P2n = np.hstack([R, T.reshape(3, 1)])

        X = cv2.triangulatePoints(P1n, P2n,
                                  n1.reshape(2, 1), n2.reshape(2, 1))
        X = X[:3] / X[3]
        return X.flatten()

    X1 = triangulate(map1[tar1], map2[tar1])
    X2 = triangulate(map1[tar2], map2[tar2])

    measured_dist = np.linalg.norm(X1 - X2)
    print(f'期望距离：{expected_dist * 1000}mm')
    print(f"三角测量距离: {measured_dist * 1000:.3f} mm")
    print(f"误差: {(measured_dist - expected_dist) * 1000:.3f} mm")


if __name__=='__main__':
    test_measure()









