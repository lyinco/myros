#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from collections import deque
import os
import json

from base_utils import find_charuco_corners


class StereoProbe:
    def __init__(self, tar_idx = [2, 87], expected_dist = 0.15945):
        ros_measure_folder = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        with open(os.path.join(ros_measure_folder, 'online_calib/src/online_calib/test.json')) as f:
            self.calib_data = json.load(f)

        self.tar_idx = tar_idx
        self.expected_dist = expected_dist
        self.Pose0 = np.hstack([np.eye(3), np.zeros((3, 1))])  # 左目：R=I, t=0
        self.Pose1 = np.hstack([np.array(self.calib_data['R']), np.array(self.calib_data['T']).reshape(3, 1)])  # 右目：R, t


        self.image_total_cnt = 0
        self.image_proc_cnt = 0

        self.bridge = CvBridge()
        rospy.Subscriber('/image/raw/cam0', Image, self.cb0)
        rospy.Subscriber('/image/raw/cam1', Image, self.cb1)
        self.pub = rospy.Publisher('/stereo_probe/points3d', PointCloud, queue_size=1)

        self.K0 = np.array([[971.6, 0, 603.8], [0, 979.7, 476.6], [0, 0, 1]])
        self.D0 = np.array([-0.0684, 0.2406, -0.00086, 0.00061])
        self.K1 = self.K0.copy()
        self.D1 = self.D0.copy()
        self.baseline = 0.05945

        self.buf0 = deque()
        self.buf1 = deque()
        self.last_pair = None

    def cb0(self, msg):
        self.buf0.append((msg.header.stamp, self.bridge.imgmsg_to_cv2(msg, 'mono8')))
        self.sync()

    def cb1(self, msg):
        self.buf1.append((msg.header.stamp, self.bridge.imgmsg_to_cv2(msg, 'mono8')))
        self.sync()

    def sync(self):
        while self.buf0 and self.buf1:
            t0, img0 = self.buf0[0]
            t1, img1 = self.buf1[0]
            dt = abs((t0 - t1).to_sec())
            if dt > 0.005:
                # 时间差太大，丢旧的
                if t0 < t1:
                    self.buf0.popleft()
                    self.image_total_cnt += 1
                else:
                    self.buf1.popleft()
                    self.image_total_cnt += 1
                continue
            # 配对成功
            self.buf0.popleft()
            self.buf1.popleft()
            print(f'total paired = {self.image_proc_cnt} total = {self.image_total_cnt}  time_diff = {dt} ')
            self.image_proc_cnt += 2
            self.image_total_cnt += 2
            # self.process_pair(img0, img1)
            self.last_pair = (img0, img1)

    def show(self):
        if self.last_pair is not None:
            img0, img1 = self.last_pair
            cv2.imshow('img0', img0)
            cv2.imshow('img1', img1)
            cv2.waitKey(1)
            self.last_pair = None

    def triangulate(self, px1, px2):
        p1 = np.array(px1, dtype=np.float64).reshape(1, 1, 2)
        p2 = np.array(px2, dtype=np.float64).reshape(1, 1, 2)

        # 去畸变 -> 归一化平面坐标 (x/z, y/z)
        n1 = cv2.undistortPoints(p1, np.array(self.calib_data['m1']), np.array(self.calib_data['d1']))  # 1x1x2
        n2 = cv2.undistortPoints(p2, np.array(self.calib_data['m2']), np.array(self.calib_data['d2']))

        # 用归一化坐标构造投影矩阵（去掉内参）
        P1n = np.hstack([np.eye(3), np.zeros((3, 1))])
        P2n = np.hstack([np.array(self.calib_data['R']), np.array(self.calib_data['T']).reshape(3, 1)])

        X = cv2.triangulatePoints(P1n, P2n,
                                  n1.reshape(2, 1), n2.reshape(2, 1))
        X = X[:3] / X[3]
        return X.flatten()

    def process_pair(self):

        if self.last_pair is not None:
            img0, img1 = self.last_pair
            vis0, vis1 = cv2.cvtColor(img0, cv2.COLOR_GRAY2BGR), cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
            self.last_pair = None

            # 检测角点
            corners1, idx1 = find_charuco_corners(img0, board=None )
            corners2, idx2 = find_charuco_corners(img1, board=None)
            if idx1 is not None:
                idx1 = idx1.flatten()
                dict1 = dict(zip(idx1, corners1))
            if idx2 is not None:
                idx2 = idx2.flatten()
                dict2 = dict(zip(idx2, corners2))

            if idx1 is not None and idx2 is not None and self.tar_idx[0] in idx1 and self.tar_idx[1] in idx1 and self.tar_idx[0] in idx2 and self.tar_idx[1] in idx2:

                pt11, pt12 = dict1[self.tar_idx[0]], dict1[self.tar_idx[1]]
                pt21, pt22 = dict2[self.tar_idx[0]], dict2[self.tar_idx[1]]

                X1 = self.triangulate(pt11, pt21)
                X2 = self.triangulate(pt12, pt22)
                dist = np.linalg.norm(X1 - X2)
                print(f'pt11 = {pt11} pt21 = {pt21} pt12 = {pt12} pt22 = {pt22}')
                print(f'pt1 = {X1}, pt2 = {X2}, measure = {dist} error = {dist - self.expected_dist}')

            if idx1 is not None and self.tar_idx[0] in idx1:
                pt = dict1[self.tar_idx[0]][0]
                cv2.circle(vis0, (int(pt[0]), int(pt[1])), 3, (0, 0, 255), 3)
            if idx1 is not None and self.tar_idx[1] in idx1:
                pt = dict1[self.tar_idx[1]][0]
                cv2.circle(vis0, (int(pt[0]), int(pt[1])), 3, (0, 0, 255), 3)
            if idx2 is not None and self.tar_idx[0] in idx2:
                pt = dict2[self.tar_idx[0]][0]
                cv2.circle(vis1, (int(pt[0]), int(pt[1])), 3, (0, 0, 255), 3)
            if idx2 is not None and self.tar_idx[1] in idx2:
                pt = dict2[self.tar_idx[1]][0]
                cv2.circle(vis1, (int(pt[0]), int(pt[1])), 3, (0, 0, 255), 3)

            h, w = img0.shape[:2]
            cv2.imshow('img0', cv2.resize(vis0, (w//2, h//2)))
            cv2.imshow('img1', cv2.resize(vis1, (w//2, h//2)))
            cv2.waitKey(1)
            self.last_pair = None


if __name__ == '__main__':
    rospy.init_node('stereo_probe')
    # StereoProbe()
    # rospy.spin()
    node = StereoProbe()
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        node.process_pair()  # ← 主线程里 imshow
        rate.sleep()