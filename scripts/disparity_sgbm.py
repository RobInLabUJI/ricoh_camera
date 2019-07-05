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

    #imgL = rotate_ccw_90(top_panorama)
    #imgR = rotate_ccw_90(btm_panorama)
    imgL = rotate_cw_90(btm_panorama)
    imgR = rotate_cw_90(top_panorama)
    #subimgL = imgL[:,64:576]
    #subimgR = imgR[:,64:576]
    subimgL = imgL
    subimgR = imgR

    window_size = 3
    min_disp = 16
    num_disp = 112-min_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 5,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )
    disp = stereo.compute(subimgL, subimgR).astype(np.float32) #/ 16.0
    output = rotate_ccw_90((disp-min_disp)/num_disp)
    #output = np.roll(output, -dx, axis=1)

    image_message = bridge.cv2_to_imgmsg(output, encoding="passthrough")

    #output = rotate_ccw_90(cv2.addWeighted(subimgL,0.5,subimgR,0.5,0))
    #image_message = bridge.cv2_to_imgmsg(output, encoding="bgr8")
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
