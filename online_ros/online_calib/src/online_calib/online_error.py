import json
import cv2
import numpy as np
from calibration import get_charuco_board, find_charuco_corners, charuco_physical_distance

calib_file = r'./test.json'

avail_size = [(1920,600, 60), (2560,800, 60), (2560,720, 60), (1280, 400, 120), (1280,720,120)]
tar_size = avail_size[2][:2] #(1920, 600)  #
tar_idx = [2, 87]
board = get_charuco_board()
expected_dist = charuco_physical_distance(board, tar_idx[0], tar_idx[1])
expected_dist_mm = 1000*expected_dist
print('expected dist = {0} mm'.format(expected_dist_mm))



with open(calib_file) as f:
    calib_data = json.load(f)
m1 = np.array(calib_data["m1"])
m2 = np.array(calib_data["m2"])
d1 = np.array(calib_data["d1"])
d2 = np.array(calib_data["d2"])
R = np.array(calib_data["R"])
T = np.array(calib_data["T"])
ret = calib_data["ret"]

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


cap  = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print('相机未打开')
    exit()

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, tar_size[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, tar_size[1])
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

fourcc = cap.get(cv2.CAP_PROP_FOURCC)
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f'actual_format = {fourcc} actual_w = {actual_w} actual_h = {actual_h}')
if actual_w != tar_size[0] or actual_h != tar_size[1]:
    print(f'actual_w = {actual_w} actual_h = {actual_h}')


while True:
    ret, frame = cap.read()
    if ret:
        h, w = frame.shape[:2]
        vis_left = frame[:, :w // 2]
        vis_right = frame[:, w // 2:]
        left = cv2.cvtColor(vis_left, cv2.COLOR_BGR2GRAY)
        right = cv2.cvtColor(vis_right, cv2.COLOR_BGR2GRAY)

        charuco_corners1, charuco_ids1 = find_charuco_corners(left, board)
        charuco_corners2, charuco_ids2 = find_charuco_corners(right, board)
        if charuco_ids1 is not None and charuco_ids2 is not None:
            charuco_ids1 = charuco_ids1.flatten()
            charuco_ids2 = charuco_ids2.flatten()
            dict1 = dict(zip(charuco_ids1, charuco_corners1))
            dict2 = dict(zip(charuco_ids2, charuco_corners2))

            if tar_idx[0] in charuco_ids1:
                pt = dict1[tar_idx[0]][0]
                vis_left = cv2.circle(vis_left, (int(pt[0]), int(pt[1])) , 5, (0, 0, 255), -1)
            if tar_idx[1] in charuco_ids1:
                pt = dict1[tar_idx[1]][0]
                vis_left = cv2.circle(vis_left, (int(pt[0]), int(pt[1])) , 5, (0, 0, 255), -1)
            if tar_idx[0] in charuco_ids2:
                pt = dict2[tar_idx[0]][0]
                vis_right = cv2.circle(vis_right, (int(pt[0]), int(pt[1])) , 5, (0, 0, 255), -1)
            if tar_idx[1] in charuco_ids2:
                pt = dict2[tar_idx[1]][0]
                vis_right = cv2.circle(vis_right, (int(pt[0]), int(pt[1])) , 5, (0, 0, 255), -1)

            if tar_idx[0] in charuco_ids1 and tar_idx[1] in charuco_ids1 and tar_idx[0] in charuco_ids2 and tar_idx[1] in charuco_ids2:
                pt11, pt12  = dict1[tar_idx[0]], dict1[tar_idx[1]]
                pt21, pt22 = dict2[tar_idx[0]], dict2[tar_idx[1]]
                pt1 = triangulate(pt11, pt21)
                pt2 = triangulate(pt12, pt22)
                measured_dist = np.linalg.norm(pt1 - pt2)
                measured_dist_mm = measured_dist * 1000
                mes_error = measured_dist_mm - expected_dist_mm
                vis_left = cv2.putText(vis_left, f'{round(mes_error, 5)}', (100, 100),  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow('left', cv2.resize(vis_left, (w // 4, h // 2)))
        cv2.imshow('right', cv2.resize(vis_right, (w // 4, h // 2)))
        keyv = cv2.waitKey(1)
        if keyv == ord('q') or keyv == ord('Q'):
            break

