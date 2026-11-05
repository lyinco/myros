#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cam_index = 0
# v4l2-ctl -d /dev/video0 --list-formats-ext
cam_size = (2560,720,60) # （2560，800,60） (2560,720,60), (1280,800,120)(1280,720,120)
stereo_mode = False
topic_cam0_raw = '/image/raw/cam0'
topic_cam1_raw = '/image/raw/cam1'
images_topics = [topic_cam0_raw, topic_cam1_raw]


class StereoCamerasPublisher:
    def __init__(self, cam_index: int = 0, tar_fps: int = 120, tar_width: int = 2560, tar_height: int = 800):
        self.bridge = CvBridge()

        # 发布处理后的图像
        self.image_pubs = [ rospy.Publisher(topic, Image, queue_size=1) for topic in images_topics]
        # self.image_pub = rospy.Publisher("/camera/processed/image_raw",
        #                                  Image,
        #                                  queue_size=1)

        # 打开摄像头：0 通常是 /dev/video0
        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头 /dev/video0")
            raise RuntimeError("Camera open failed")

        # 可选：设置分辨率
        self.tar_fps = tar_fps
        self.tar_width = tar_width // 2
        self.tar_height = tar_height

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, tar_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, tar_height)


        # 发布频率（帧率）
        self.rate = rospy.Rate(tar_fps)

    def process_image(self, frame):
        """
        在这里写你的图像处理逻辑。
        下面只是一个示例：灰度 + 边缘检测。
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        # 转回 BGR 三通道，便于以 bgr8 发布
        processed = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        return processed

    def run(self):

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("读取摄像头帧失败")
                continue

            cam0 = frame[:, :self.tar_width]
            cam1 = frame[:, self.tar_width:]


            # 2) 转成 ROS Image 并发布
            try:
                image_msgs = [self.bridge.cv2_to_imgmsg(img, encoding="bgr8") for img in [cam0, cam1]]
                # msg = self.bridge.cv2_to_imgmsg(cam0, encoding="bgr8")
            except CvBridgeError as e:
                print('error : ', str(e))
                rospy.logerr(e)
                continue

            # msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "camera"
            # self.bridge.cv2_to_imgmsg(cam0, encoding="bgr8")
            for idx in range(len(image_msgs)):
                image_msgs[idx].header.stamp = rospy.Time.now()
                msg = image_msgs[idx]
                msg.header.frame_id = f'cam{idx}'
                self.image_pubs[idx].publish(msg)

            # 3) 本地预览（可选，调试用）
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            self.rate.sleep()

        # 清理资源
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':

    rospy.init_node('camera_processor_publisher', anonymous=True)
    node = StereoCamerasPublisher(cam_index=cam_index, tar_fps=cam_size[2], tar_width=cam_size[0], tar_height=cam_size[1])
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
