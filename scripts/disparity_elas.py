#!/usr/bin/env python
# license removed for brevity

import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
import os
import time

from cv_bridge import CvBridge, CvBridgeError
from elas import *

def rotate_cw_90(img):
    img_t = cv2.transpose(img)
    img_f = cv2.flip(img_t, flipCode=1)
    return img_f

def rotate_ccw_90(img):
    img_t = cv2.transpose(img)
    img_f = cv2.flip(img_t, flipCode=0)
    return img_f

def callback(top_image, btm_image):

    try:
      top_panorama = bridge.imgmsg_to_cv2(top_image, "bgr8")
      btm_panorama = bridge.imgmsg_to_cv2(btm_image, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    #dx = int(320 * 88.35 / 90.0)
    dx = 325
    btm_panorama = np.roll(btm_panorama, dx, axis=1)

    #imgL = rotate_cw_90(top_panorama)
    #imgR = rotate_cw_90(btm_panorama)
    imgL = rotate_cw_90(btm_panorama)
    imgR = rotate_cw_90(top_panorama)
    subimgL = cv2.cvtColor(imgL[:,64:576], cv2.COLOR_BGR2GRAY)
    subimgR = cv2.cvtColor(imgR[:,64:576], cv2.COLOR_BGR2GRAY)
    #subimgL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    #subimgR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    d1 = np.empty_like(subimgL, dtype=np.float32)
    d2 = np.empty_like(subimgR, dtype=np.float32)

    params = Elas_parameters()
    params.postprocess_only_left = False
    elas = Elas(params)

    elas.process_stereo(subimgL, subimgR, d1, d2)

    output = rotate_ccw_90(d1)
    #output = (output / np.amax(output)*255)
    image_message = bridge.cv2_to_imgmsg(output, encoding="passthrough")

    #output = rotate_ccw_90(cv2.addWeighted(subimgL,0.5,subimgR,0.5,0))
    #image_message = bridge.cv2_to_imgmsg(output, encoding="mono8")
    
    image_message.header.stamp = top_image.header.stamp
    pub.publish(image_message)

    return

if __name__ == '__main__':
    rospy.init_node('disparity_sgbm', anonymous=True)
    bridge = CvBridge()
    
    top_image_sub = message_filters.Subscriber('/top/image_rect', Image)
    btm_image_sub = message_filters.Subscriber('/bottom/image_rect', Image)

    ts = message_filters.ApproximateTimeSynchronizer([top_image_sub, btm_image_sub], 10, 0.005)
    ts.registerCallback(callback)
    
    pub = rospy.Publisher('image_disparity', Image, queue_size=10)

    rospy.spin()
