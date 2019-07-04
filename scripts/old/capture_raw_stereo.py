#!/usr/bin/env python
# license removed for brevity

import rospy
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

def talker():
    global cap
    global skip
    global frequency
    global video_width
    global display

    image_size = video_width / 2

    bridge = CvBridge()
    pub1 = rospy.Publisher('cam_bottom/image_raw', Image, queue_size=10)
    pub2 = rospy.Publisher('cam_top/image_raw',    Image, queue_size=10)
    rate = rospy.Rate(frequency)
    counter = 0
    while not rospy.is_shutdown():
        cap1.grab()
        stamp1 = rospy.Time.from_sec(time.time())
        cap2.grab()
        stamp2 = rospy.Time.from_sec(time.time())
        ret, frame1 = cap1.retrieve()
        ret, frame2 = cap2.retrieve()
        counter += 1
        if counter > skip:
            image1_message = bridge.cv2_to_imgmsg(frame1[0:image_size,:], encoding="bgr8")
            image1_message.header.stamp = stamp1
            pub1.publish(image1_message)
            image2_message = bridge.cv2_to_imgmsg(frame2[0:image_size,:], encoding="bgr8")
            image2_message.header.stamp = stamp2
            pub2.publish(image2_message)
            counter = 0
        if display:
            cv2.imshow('/dev/video%d'%video_input_1, frame1[0:image_size,:])
            cv2.imshow('/dev/video%d'%video_input_2, frame2[0:image_size,:])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break 
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('capture_raw_stereo', anonymous=True)

    if rospy.has_param('~video_input_bottom'):
        video_input_1 = rospy.get_param('~video_input_bottom')
    else:
        video_input_1 = 0

    if rospy.has_param('~video_input_top'):
        video_input_2 = rospy.get_param('~video_input_top')
    else:
        video_input_2 = 1

    supported_widths = [1280, 1920]
    if rospy.has_param('~width'):
        video_width = rospy.get_param('~width')
        if not video_width in supported_widths:
            print("Unsupported width %d: valid options %s" % (video_width, supported_widths))
    else:
        video_width = supported_widths[0]

    supported_heights = [720, 1080]
    if rospy.has_param('~height'):
        video_height = rospy.get_param('~height')
        if not video_height in supported_heights:
            print("Unsupported height %d: valid options %s" % (video_height, supported_heights))
    else:
        video_height = supported_heights[0]

    if rospy.has_param('~display'):
        display = rospy.get_param('~display')
    else:
        display = False

    if rospy.has_param('~frequency'):
        frequency = rospy.get_param('~frequency')
        if frequency > 30:
            frequency = 30
    else:
        frequency = 30

    if rospy.has_param('~skip'):
        skip = rospy.get_param('~skip')
    else:
        skip = 0

    cap1 = cv2.VideoCapture(video_input_1)
    cap2 = cv2.VideoCapture(video_input_2)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT,video_height)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT,video_height)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
