import os.path
import pickle
import cv2
from typing import Optional, List, Tuple
from pathlib import Path
import numpy as np
from glob import glob

from calibration import *

def main_multi2(m1, d1, m2, d2, image_dir = r'F:\git_src\usb_grab\images\cam1_cam2_0910_144240'):

    files = glob(os.path.join(image_dir, 'left*.png'))
    indx = [ os.path.basename(f).replace('.png', '').split('_')[-1] for f in files]
    indx = [int(d) for d in indx]
    indx.sort()
    print(indx)

    board = get_charuco_board()

    left_pts, right_pts, objpts = [],[], []
    h, w = None, None
    for img_idx in indx:
        file1 = os.path.join(image_dir, f'left_{img_idx}.png')
        file2 = os.path.join(image_dir, f'right_{img_idx}.png')
        image1 = cv2.imread(file1, 0)
        image2 = cv2.imread(file2, 0)
        h, w = image1.shape[:2]
        chcorners1, chcids1 = find_charuco_corners(image1, board)
        chcorners2, chcids2 = find_charuco_corners(image2, board)
        if chcids1 is None or chcids2 is None:
            continue

        chcids1 = chcids1.flatten()
        chcids2 = chcids2.flatten()
        shared_idx = list(set(chcids1).intersection(set(chcids2)))
        if len(shared_idx) == 0:
            continue

        shared_idx.sort()
        dict1 = dict(zip(chcids1, chcorners1))
        dict2 = dict(zip(chcids2, chcorners2))
        print(f'image shared counts = {len(shared_idx)}')

        per_image_left, per_image_right, per_image_obj = [], [], []
        for midx in shared_idx:
            pt1 = dict1[midx]
            pt2 = dict2[midx]
            per_image_left.append(pt1)
            per_image_right.append(pt2)
        per_image_obj, _ = board.matchImagePoints(per_image_left, np.array([[ii] for ii in shared_idx]))

        left_pts.append(np.array(per_image_left))
        right_pts.append(np.array(per_image_right))
        objpts.append(np.array(per_image_obj))

    print('start run core stereo calibrate')
    retval, m1, d1, m2, d2, R, T, E, F = cv2.stereoCalibrate(objpts, left_pts, right_pts,
                                                                 m1, d1, m2, d2,(w, h),
                                                                 R=None, T=None,
                                                                 E=None, F=None,
                                                                 flags=cv2.CALIB_FIX_INTRINSIC,
                                                                 criteria=(
                                                                     cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                                                                     30, 1e-6))

    print('stereo done. ')
    return m1, d1, m2, d2, R, T


def self_validatation():
    import pickle
    image_dir = r'F:\git_src\usb_grab\images\zhijian_0910_105651'
    cam_pkl = r'F:\git_src\usb_grab\calibs\quality.pkl'
    with open(cam_pkl, 'rb') as f:  # 必须以 'rb' 二进制模式打开
        calib_data = pickle.load(f)
        m1 = calib_data['m1']
        d1 = calib_data['d1']
        m2 = calib_data['m2']
        d2 = calib_data['d1']
        T_org = calib_data['T']

    m1_new, d1_new, m2_new, d2_new, R_new, T_new = main_multi2(m1, d1, m2, d2, image_dir)
    print(f'T_org = {T_org.tolist()}')
    print(f'T_new = {T_new.tolist()}')
    print(f'm1_org = {m1.tolist()}')
    print(f'm1_new = {m1_new.tolist()}')
    print(f'm2_org = {m2.tolist()}')
    print(f'm2_new = {m2_new.tolist()}')


def multi_calib():
    image_dir = r'F:\git_src\usb_grab\images\cam1_cam3_0911_113926'
    cam1_pkl = r'F:\git_src\usb_grab\calibs\quality.pkl'
    cam2_pkl = r'F:\git_src\usb_grab\calibs\xinyue.pkl'
    with open(cam1_pkl, 'rb') as f:  # 必须以 'rb' 二进制模式打开
        calib_data1 = pickle.load(f)
        m1 = calib_data1['m1']
        d1 = calib_data1['d1']

    with open(cam2_pkl, 'rb') as f:  # 必须以 'rb' 二进制模式打开
        calib_data2 = pickle.load(f)
        m2 = calib_data2['m1']
        d2 = calib_data2['d1']

    m1_new, d1_new, m2_new, d2_new, R_new, T_new = main_multi2(m1, d1, m2, d2, image_dir)
    print(f'm1_org = {m1.tolist()}')
    print(f'm1_new = {m1_new.tolist()}')

    print('cam1-cam2 result: ')
    print(f'R = {R_new.tolist()}')
    print(f'T = {T_new.tolist()}')


if __name__=='__main__':
    # self_validatation()
    multi_calib()



















