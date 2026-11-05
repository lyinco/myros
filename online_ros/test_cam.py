



import cv2

cam_size = (2560, 800)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_size[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_size[1])
# cap.set(cv2.CAP_PROP_FPS, 120)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f'actual size = {actual_w} x {actual_h} pixels')
half_size = actual_w // 2


if cap.isOpened() is False:
    print("Error opening video stream or file")
    exit(0)


while True:
    ret, frame = cap.read()
    if ret:
        left = frame[:, :half_size]
        right = frame[:, half_size:]
        cv2.imshow("left", left)
        cv2.imshow("right", right)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print('error when read image')
        break