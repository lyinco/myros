import pickle
import cv2
import numpy as np



def get_charuco_board(squares_x =12, squares_y =9, square_length =0.015, marker_length =0.01175, dict_type = cv2.aruco.DICT_5X5_1000):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length,
        marker_length,
        dictionary
    )
    return board


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


# def find_charuco_corners(gray, board):
#     detector = cv2.aruco.CharucoDetector(board)
#     charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
#     if charuco_ids is not None and len(charuco_ids) > 0:
#         return charuco_corners, charuco_ids
#     return None, None

def find_charuco_corners(gray, board, win=(5, 5), max_iter=50, eps=1e-4):

    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
    if charuco_ids is None or len(charuco_ids) == 0:
        return None, None

    corners = charuco_corners.reshape(-1, 1, 2).astype(np.float32)
    cv2.cornerSubPix(
        gray, corners, win, (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iter, eps))
    return corners.reshape(-1, 1, 2), charuco_ids



def get_measure_distance(tar1, tar2, image1, image2, calib_data, board      ):
    charuco_corners1, charuco_ids1 = find_charuco_corners(image1, board)
    if charuco_corners1 is None or charuco_ids1 is None or len(charuco_ids1) == 0 :
        return

    charuco_corners2, charuco_ids2 = find_charuco_corners(image2, board)
    if charuco_corners2 is None or charuco_ids2 is None or len(charuco_ids2) == 0 :
        return

    vis = image1.copy()
    cv2.aruco.drawDetectedCornersCharuco(
        vis,
        charuco_corners1,
        charuco_ids1,
        cornerColor=(0, 0, 255)  # 红色角点
    )
    cv2.imshow('markers', vis)
    cv2.waitKey(1)

    map1 = dict(zip(charuco_ids1.flatten(), charuco_corners1))
    map2 = dict(zip(charuco_ids2.flatten(), charuco_corners2))
    if tar1 not in map1 or tar2 not in map1 or tar1 not in map2 or tar2 not in map2:
        return


    m1 = calib_data['m1']
    m2 = calib_data['m2']
    d1 = calib_data['d1']
    d2 = calib_data['d2']
    R = calib_data['R']
    T = calib_data['T']

    # 构造投影矩阵
    # P1 = m1 @ np.hstack([np.eye(3), np.zeros((3, 1))])
    # P2 = m2 @ np.hstack([R, T.reshape(3, 1)])

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
    return measured_dist


def init_cam(cam_index: int = 0, tar_size: list = [2580, 800]):

    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, tar_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, tar_size[1])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if actual_w != tar_size[0] or actual_h != tar_size[1]:
        print(f'actual_w = {actual_w} actual_h = {actual_h}')

    return cap


def main(cam_index, tar_size: list = [2560, 800]  ):

    calib_file = r'test.pkl'
    with open(calib_file, 'rb') as f:
        calib_data = pickle.load(f)

    tar_charucos = [12, 64]
    board = get_charuco_board()

    expected_dist = charuco_physical_distance(board, tar_charucos[0], tar_charucos[1])
    expected_dist_mm = 1000 * expected_dist
    print('expected_dist_mm = ', expected_dist_mm)


    cap = init_cam(cam_index=cam_index, tar_size=tar_size)
    if cap is None:
        raise Exception('Camera not available')

    while True:
        ret, frame = cap.read()
        if ret:
            left = frame[:, :tar_size[0]//2 ]
            right = frame[:, tar_size[0] // 2:]
            measured_dist = get_measure_distance(tar_charucos[0], tar_charucos[1], left, right, calib_data, board)
            if measured_dist is not None:
                measured_dist_mm = 1000 * measured_dist
                print(f'error = {measured_dist_mm - expected_dist_mm} expected_dist_mm =  {expected_dist_mm} measured_dist_mm = {measured_dist_mm} ')

            cv2.imshow('left', left)
            cv2.imshow('right', right)
            keyv = cv2.waitKey(1)
            if keyv == 27 or keyv == ord('q') or keyv == ord('Q'):
                break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    cam_index = 1
    tar_size = [2580, 800]
    main(cam_index=cam_index, tar_size=tar_size)

