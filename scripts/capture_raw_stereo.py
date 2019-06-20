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
    global split
    global frequency
    global video_width
    global display

    image_size = video_width / 2

    bridge = CvBridge()
    if split:
        pub_f1 = rospy.Publisher('cam_bottom/image_front_raw', Image, queue_size=10)
        pub_b1 = rospy.Publisher('cam_bottom/image_back_raw', Image, queue_size=10)
        pub_f2 = rospy.Publisher('cam_top/image_front_raw', Image, queue_size=10)
        pub_b2 = rospy.Publisher('cam_top/image_back_raw', Image, queue_size=10)
    else:
        pub1 = rospy.Publisher('cam_bottom/image_raw', Image, queue_size=10)
        pub2 = rospy.Publisher('cam_top/image_raw', Image, queue_size=10)
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        cap1.grab()
        stamp1 = rospy.Time.from_sec(time.time())
        cap2.grab()
        stamp2 = rospy.Time.from_sec(time.time())
        ret, frame1 = cap1.retrieve()
        ret, frame2 = cap2.retrieve()
        front1 = frame1[0:image_size, 0:image_size]
        back1  = frame1[0:image_size, image_size:2*image_size]
        output1 = np.concatenate( (front1, back1), axis=1 )
        front2 = frame2[0:image_size, 0:image_size]
        back2  = frame2[0:image_size, image_size:2*image_size]
        output2 = np.concatenate( (front2, back2), axis=1 )
        if display:
            cv2.imshow('/dev/video%d'%video_input_1, output1)
            cv2.imshow('/dev/video%d'%video_input_2, output2)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        if split:
            image_f_message = bridge.cv2_to_imgmsg(front1, encoding="bgr8")
            image_b_message = bridge.cv2_to_imgmsg(back1,  encoding="bgr8")
            image_f_message.header.stamp = stamp1
            image_b_message.header.stamp = stamp1
            pub_f1.publish(image_f_message)
            pub_b1.publish(image_b_message)
            image_f_message = bridge.cv2_to_imgmsg(front2, encoding="bgr8")
            image_b_message = bridge.cv2_to_imgmsg(back2,  encoding="bgr8")
            image_f_message.header.stamp = stamp2
            image_b_message.header.stamp = stamp2
            pub_f2.publish(image_f_message)
            pub_b2.publish(image_b_message)
        else:
            image_message = bridge.cv2_to_imgmsg(output1, encoding="bgr8")
            image_message.header.stamp = stamp1
            pub1.publish(image_message)
            image_message = bridge.cv2_to_imgmsg(output1, encoding="bgr8")
            image_message.header.stamp = stamp2
            pub2.publish(image_message)
 
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
