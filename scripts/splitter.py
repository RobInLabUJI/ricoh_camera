#!/usr/bin/env python
# license removed for brevity

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def rotate_cw_90(img):
    img_t = cv2.transpose(img)
    img_f = cv2.flip(img_t, flipCode=1)
    return img_f

def rotate_ccw_90(img):
    img_t = cv2.transpose(img)
    img_f = cv2.flip(img_t, flipCode=0)
    return img_f

def callback(data):
    global invert

    try:
      image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    image_size = image.shape[0]
    if invert:
        front = rotate_ccw_90(image[0:image_size, 0:image_size])
        back  = rotate_cw_90(image[0:image_size, image_size:2*image_size])
    else:
        front = rotate_cw_90(image[0:image_size, 0:image_size])
        back  = rotate_ccw_90(image[0:image_size, image_size:2*image_size])
    image_f_message = bridge.cv2_to_imgmsg(front, encoding="bgr8")
    image_b_message = bridge.cv2_to_imgmsg(back,  encoding="bgr8")
    image_f_message.header.stamp = data.header.stamp
    image_b_message.header.stamp = data.header.stamp
    pub_f.publish(image_f_message)
    pub_b.publish(image_b_message)

if __name__ == '__main__':
    rospy.init_node('splitter', anonymous=True)

    if rospy.has_param('~invert'):
        invert = rospy.get_param('~invert')
    else:
        invert = False

    bridge = CvBridge()
    sub = rospy.Subscriber("image_raw", Image, callback)
    pub_f = rospy.Publisher('front/image_raw', Image, queue_size=10)
    pub_b = rospy.Publisher('back/image_raw',  Image, queue_size=10)

    rospy.spin()
