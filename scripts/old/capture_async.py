#!/usr/bin/env python
# license removed for brevity

import rospy
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from VideoCaptureAsync import VideoCaptureAsync

import numpy as np
import cv2

def talker():
    global cap
    global split
    global frequency
    global video_width
    global display

    image_size = video_width / 2

    bridge = CvBridge()
    if split:
        pub_f = rospy.Publisher('image_front_raw', Image, queue_size=10)
        pub_b = rospy.Publisher('image_back_raw', Image, queue_size=10)
    else:
        pub = rospy.Publisher('image_raw', Image, queue_size=10)
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        ret, frame, stamp = cap.read()
        front = frame[0:image_size, 0:image_size]
        front = cv2.transpose(front)
        front = cv2.flip(front, flipCode=1)
        back  = frame[0:image_size, image_size:2*image_size]
        back = cv2.transpose(back)
        back = cv2.flip(back, flipCode=0)
        output = np.concatenate( (front, back), axis=1 )
        if display:
            cv2.imshow('/dev/video%d'%video_input, output)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        if split:
            image_f_message = bridge.cv2_to_imgmsg(front, encoding="bgr8")
            image_b_message = bridge.cv2_to_imgmsg(back,  encoding="bgr8")
            image_f_message.header.stamp = stamp
            image_b_message.header.stamp = stamp
            pub_f.publish(image_f_message)
            pub_b.publish(image_b_message)
        else:
            image_message = bridge.cv2_to_imgmsg(output, encoding="bgr8")
            image_message.header.stamp = stamp
            pub.publish(image_message)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('capture', anonymous=True)

    if rospy.has_param('~video_input'):
        video_input = rospy.get_param('~video_input')
    else:
        video_input = 0

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

    if rospy.has_param('~split'):
        split = rospy.get_param('~split')
    else:
        split = False

    if rospy.has_param('~display'):
        display = rospy.get_param('~display')
    else:
        display = False

    if rospy.has_param('~frequency'):
        frequency = rospy.get_param('~frequency')
        if frequency > 30:
            frequency = 30
    else:
        frequency = 15

    cap = VideoCaptureAsync(video_input)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,video_height)
    cap.start()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    cap.stop()
    cv2.destroyAllWindows()

