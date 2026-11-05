import os.path

import cv2
import time
from datetime import datetime

'''
	[0]: 'MJPG' (Motion-JPEG, compressed)
		Size: Discrete 2560x800
			Interval: Discrete 0.017s (60.000 fps)
		Size: Discrete 2560x720
			Interval: Discrete 0.017s (60.000 fps)
		Size: Discrete 1280x800
			Interval: Discrete 0.008s (120.000 fps)
		Size: Discrete 1280x792
			Interval: Discrete 0.008s (120.000 fps)
		Size: Discrete 1280x400
			Interval: Discrete 0.008s (120.000 fps)
		Size: Discrete 640x400
			Interval: Discrete 0.005s (210.000 fps)
		Size: Discrete 640x392
			Interval: Discrete 0.005s (210.000 fps)
		Size: Discrete 1280x480
			Interval: Discrete 0.017s (60.000 fps)

'''

avail_size = [(2560,800, 60), (2560,720, 60), (1280, 400, 120), (1280,720,120)]
tar_size = avail_size[1][:2] #(1920, 600)  #
cap  = cv2.VideoCapture(1, cv2.CAP_DSHOW)
if not cap.isOpened():
    print('相机未打开')
    exit()


def grab_images(stereo_image_dir):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, tar_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, tar_size[1])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    fourcc = cap.get(cv2.CAP_PROP_FOURCC)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'actual_format = {fourcc} actual_w = {actual_w} actual_h = {actual_h}')
    if actual_w != tar_size[0] or actual_h != tar_size[1]:
        print(f'actual_w = {actual_w} actual_h = {actual_h}')




    for i in range(5):
        cap.read()


    frame_id = 0
    grab_num = 0
    start_time = time.time()
    date_str = datetime.fromtimestamp(start_time).strftime('%m%d_%H%M%S')
    out_dir = os.path.join(stereo_image_dir, date_str)
    os.makedirs(out_dir, exist_ok=True)

    while True:
        success, frame = cap.read()
        frame_id = frame_id + 1
        if success:
            h, w = frame.shape[:2]
            left = frame[:,:w//2]
            right = frame[:,w//2:]
            cv2.imshow('left', cv2.resize(left, (w , h*2), interpolation=cv2.INTER_AREA))
            cv2.imshow('right', cv2.resize(right, (w, h*2), interpolation=cv2.INTER_AREA))
            # cv2.imshow('left', cv2.resize(left, (w//2, h)))
            # cv2.imshow('right', cv2.resize(right, (w//2, h)))
            keyv = cv2.waitKey(1)
            if keyv == ord('s') or keyv == ord('S'):
                grab_num+=1
                file1 = os.path.join(out_dir, f'left_{grab_num}.png')
                file2 = os.path.join(out_dir, f'right_{grab_num}.png')
                cv2.imwrite(file1, left)
                cv2.imwrite(file2, right)

            if keyv == ord('q') or keyv==ord('Q'):
                cap.release()
                cv2.destroyAllWindows()
                break

    elapsed = time.time() - start_time
    fps = frame_id / elapsed
    print(f'fps = {fps}')

if __name__ == '__main__':
    grab_images('images')
