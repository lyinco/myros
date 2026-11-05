#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def callback(msg):
    # 计算延迟：当前ROS时间 - 图像时间戳
    delay = (rospy.Time.now() - msg.header.stamp).to_sec()
    print(f"帧延迟: {delay:.3f} 秒")

if __name__ == '__main__':
    rospy.init_node('delay_checker')
    # 确保队列足够大，否则可能因为订阅者自身的队列丢弃而看不到真实积压
    rospy.Subscriber("/image/raw/cam0", Image, callback, queue_size=100, buff_size=2**24)
    rospy.spin()