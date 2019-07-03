#!/usr/bin/env python
# license removed for brevity

import rospy
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#import numpy as np
import cv2

def talker():
    global cap
    global frequency
    global video_resolution

    image_size = video_resolution / 2

    bridge = CvBridge()
    pub = rospy.Publisher('image_raw', Image, queue_size=10)
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        cap.grab()
        stamp = rospy.Time.from_sec(time.time())
        ret, frame = cap.retrieve()
        image  = frame[0:image_size, :]
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_message.header.stamp = stamp
        pub.publish(image_message)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('capture', anonymous=True)

    if rospy.has_param('~video_input'):
        video_input = rospy.get_param('~video_input')
    else:
        video_input = 0

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

    cap = cv2.VideoCapture(video_input)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_resolution)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,video_height[video_resolution])
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    cap.release()
    cv2.destroyAllWindows()
