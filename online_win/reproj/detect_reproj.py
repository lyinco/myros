import cv2
import numpy as np
import json

def get_charuco_board(squares_x =12, squares_y =9, square_length =0.015, marker_length =0.01175, dict_type = cv2.aruco.DICT_5X5_1000):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length,
        marker_length,
        dictionary
    )
    return board


def find_charuco_corners(gray, board, win=(5, 5), max_iter=50, eps=1e-4):

    if gray is None:
        return None, None
    if len(gray.shape) == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
    if charuco_ids is None or len(charuco_ids) == 0:
        return None, None

    corners = charuco_corners.reshape(-1, 1, 2).astype(np.float32)
    cv2.cornerSubPix(
        gray, corners, win, (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iter, eps))
    return corners.reshape(-1, 1, 2), charuco_ids.reshape(-1)



def triangulate(px1, px2, m1, d1, m2, d2, R, T):

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

def reproject(pt3d, m1, d1, m2, d2, R, T):
    points1 = cv2.projectPoints(np.array([pt3d]), rvec=np.eye(3), tvec=np.zeros(3), cameraMatrix=m1, distCoeffs=d1)
    points2 = cv2.projectPoints(np.array([pt3d]), rvec=R, tvec=T, cameraMatrix=m2, distCoeffs=d2)
    points1 = points1[0].reshape([-1, 2])
    points2 = points2[0].reshape([-1, 2])

    return points1, points2


def drawPoints(vis, uvs, color, radius):
    corners = uvs.reshape(-1, 2)
    for pt in corners:
        cv2.circle(vis, (int(pt[0]), int(pt[1])), radius, color, 2)
    return vis



if __name__ == '__main__':
    image1 = cv2.imread(r'E:\shares\git\online_win\images\1015_133730\left_1.png', cv2.IMREAD_COLOR)
    image2 = cv2.imread(r'E:\shares\git\online_win\images\1015_133730\right_1.png', cv2.IMREAD_COLOR)
    calib_file = r'E:\shares\git\online_win\test.json'
    board = get_charuco_board()
    calib_data = json.load(open(calib_file, 'r'))
    m1, d1, m2, d2, R, T = calib_data['m1'], calib_data['d1'], calib_data['m2'], calib_data['d2'], calib_data['R'], calib_data['T']
    m1, d1, m2, d2, R, T = np.array(m1), np.array(d1), np.array(m2), np.array(d2), np.array(R), np.array(T)

    corners1, ids1 = find_charuco_corners(image1, board)
    corners2, ids2 = find_charuco_corners(image2, board)
    dict1 = dict(zip(ids1, corners1))
    dict2 = dict(zip(ids2, corners2))
    shared_idx = list(set(ids1).intersection(set(ids2)))
    corners3d = [triangulate(dict1[idx], dict2[idx], m1, d1, m2, d2, R, T) for idx in shared_idx ]
    reproj_uvs = [reproject(pt, m1, d1, m2, d2, R, T) for pt in corners3d]
    reproj_left = np.array([item[0] for item in reproj_uvs])
    reproj_right = np.array([item[1] for item in reproj_uvs])

    dict_reproj1 = dict(zip(shared_idx, reproj_left))
    dict_reproj2 = dict(zip(shared_idx, reproj_right))
    dict_3d = dict(zip(shared_idx, corners3d))

    vis1 = image1.copy()
    vis2 = image2.copy()
    vis1 = drawPoints(vis1, corners1[shared_idx], (0, 255, 0), 5)
    vis2 = drawPoints(vis2, corners2[shared_idx], (0, 255, 0), 5)
    vis1 = drawPoints(vis1, reproj_left, (0, 0, 255), 3)
    vis2 = drawPoints(vis2, reproj_right, (0, 0, 255), 3)

    h, w = image1.shape[:2]
    cv2.imshow('vis1', vis1)
    cv2.imshow('vis2', vis2)
    cv2.waitKey(0)




    for idx in shared_idx:
        print(f'm_{idx}:left={dict1[idx]} right={dict2[idx]} 3d={dict_3d[idx]}')
        # print(f'm_{idx}: left_dis({dict_reproj1[idx] - dict1[idx]}) right_dis({dict_reproj2[idx] - dict2[idx]})')







