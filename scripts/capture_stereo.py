#!/usr/bin/env python
# license removed for brevity

import rospy
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

def talker():
    global cap
    global frequency
    global video_resolution
    
    image_size = video_resolution / 2

    bridge = CvBridge()
    pub1 = rospy.Publisher('bottom/image_raw', Image, queue_size=10)
    pub2 = rospy.Publisher('top/image_raw',    Image, queue_size=10)
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        cap1.grab()
        stamp1 = rospy.Time.from_sec(time.time())
        cap2.grab()
        stamp2 = rospy.Time.from_sec(time.time())
        ret, frame1 = cap1.retrieve()
        ret, frame2 = cap2.retrieve()
        image1_message = bridge.cv2_to_imgmsg(frame1[0:image_size,:], encoding="bgr8")
        image1_message.header.stamp = stamp1
        pub1.publish(image1_message)
        image2_message = bridge.cv2_to_imgmsg(frame2[0:image_size,:], encoding="bgr8")
        image2_message.header.stamp = stamp2
        pub2.publish(image2_message)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('capture_stereo', anonymous=True)

    if rospy.has_param('~video_input_bottom'):
        video_input_1 = rospy.get_param('~video_input_bottom')
    else:
        video_input_1 = 0

    if rospy.has_param('~video_input_top'):
        video_input_2 = rospy.get_param('~video_input_top')
    else:
        video_input_2 = 1

    supported_resolutions = [1280, 1920]
    if rospy.has_param('~resolution'):
        video_resolution = rospy.get_param('~resolution')
        if not video_resolution in supported_resolutions:
            print("Unsupported resolution %d: valid options %s" % (video_resolution, supported_resolutions))
            video_resolution = supported_resolutions[0]
            print("Using default: %d" % video_resolution)
    else:
        video_resolution = supported_resolutions[0]

    if rospy.has_param('~frequency'):
        frequency = rospy.get_param('~frequency')
        if frequency < 1 or frequency > 30:
            print("Unsupported frequency %d: valid interval [1, 30]" % frequency)
            frequency = 30
            print("Using default: %d" % frequency)
    else:
        frequency = 30

    video_height = {1280: 720, 1920: 1080}

    cap1 = cv2.VideoCapture(video_input_1)
    cap2 = cv2.VideoCapture(video_input_2)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, video_resolution)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT,video_height[video_resolution])
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, video_resolution)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT,video_height[video_resolution])
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
