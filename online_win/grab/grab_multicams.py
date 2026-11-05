import os.path

import cv2
import time
from datetime import datetime

tar_size = (1920, 600)

cap1  = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap2 = cv2.VideoCapture(2, cv2.CAP_DSHOW)
if not cap1.isOpened() or not cap2.isOpened():
    print('相机未打开')
    exit()


def grab_images(stereo_image_dir):
    for cap in [cap1, cap2]:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, tar_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, tar_size[1])
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_w != tar_size[0] or actual_h != tar_size[1]:
            print(f'actual_w = {actual_w} actual_h = {actual_h}')


        for i in range(5):
            cap1.read()
            cap2.read()

    frame_id = 0
    grab_num = 0
    start_time = time.time()
    date_str = datetime.fromtimestamp(start_time).strftime('%m%d_%H%M%S')
    out_dir = os.path.join(stereo_image_dir, date_str)
    os.makedirs(out_dir, exist_ok=True)

    while True:
        success1, frame1 = cap1.read()
        success2, frame2 = cap2.read()
        frame_id = frame_id + 1
        if success1 and success2:
            h, w = frame1.shape[:2]
            left1 = frame1[:,:w//2]
            right1 = frame1[:,w//2:]
            left2 = frame2[:, :w // 2]
            right2 = frame2[:, w // 2:]
            cv2.imshow('left1', cv2.resize(left1, (w//4, h//2)))
            cv2.imshow('left2', cv2.resize(left2, (w//4, h//2)))
            keyv = cv2.waitKey(1)
            if keyv == ord('s') or keyv == ord('S'):
                grab_num+=1
                file1 = os.path.join(out_dir, f'left_{grab_num}.png')
                file2 = os.path.join(out_dir, f'right_{grab_num}.png')
                cv2.imwrite(file1, left1)
                cv2.imwrite(file2, left2)

            if keyv == ord('q') or keyv==ord('Q'):
                cap.release()
                cv2.destroyAllWindows()
                break

    elapsed = time.time() - start_time
    fps = frame_id / elapsed
    print(f'fps = {fps}')

if __name__ == '__main__':
    grab_images('images')
