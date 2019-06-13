#!/usr/bin/env python
# license removed for brevity

import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def findMatches(img_top, img_btm, max_dist=50, display=None):
    orb = cv2.ORB_create()
    kpt = orb.detect(img_top, None)
    kpt, dest = orb.compute(img_top, kpt)
    kpb = orb.detect(img_btm, None)
    kpb, desb = orb.compute(img_btm, kpb)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(dest, desb)
    matches = [m for m in matches if m.distance<max_dist]
    matches = sorted(matches, key = lambda x:x.distance)

    if display is None:
    	pass
    else:
	    img_disp = cv2.drawMatches(img_top, kpt, img_btm, kpb, matches, None, flags=2)
	    cv2.imshow(display, img_disp)

def callback(top_image, btm_image):
    #print(top_image.header.stamp)
    try:
      top_panorama = bridge.imgmsg_to_cv2(top_image, "bgr8")
    except CvBridgeError as e:
      print(e)
    try:
      btm_panorama = bridge.imgmsg_to_cv2(btm_image, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    MAX_DIST = 50
    
    top_bf_stitch = cv2.cvtColor(top_panorama[:,160:480], cv2.COLOR_BGR2GRAY)
    btm_b_center  = cv2.cvtColor(btm_panorama[:,160:480], cv2.COLOR_BGR2GRAY)
    findMatches(top_bf_stitch, btm_b_center, MAX_DIST, "Top BF stitch")

    top_fb_stitch = cv2.cvtColor(top_panorama[:,800:1120], cv2.COLOR_BGR2GRAY)
    btm_f_center  = cv2.cvtColor(btm_panorama[:,800:1120], cv2.COLOR_BGR2GRAY)
    findMatches(top_fb_stitch, btm_f_center, MAX_DIST, "Top FB stitch")

    btm_bf_stitch = cv2.cvtColor(btm_panorama[:,480:800], cv2.COLOR_BGR2GRAY)
    top_f_center  = cv2.cvtColor(top_panorama[:,480:800], cv2.COLOR_BGR2GRAY)
    findMatches(top_f_center, btm_bf_stitch, MAX_DIST, "Bottom BF stitch")

    btm_fb_stitch = cv2.cvtColor(np.concatenate((btm_panorama[:,1120:1280],btm_panorama[:,0:160]), axis=1), cv2.COLOR_BGR2GRAY)
    top_b_center  = cv2.cvtColor(np.concatenate((top_panorama[:,1120:1280],top_panorama[:,0:160]), axis=1), cv2.COLOR_BGR2GRAY)
    findMatches(top_b_center, btm_fb_stitch, MAX_DIST, "Bottom FB stitch")

    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('feature_matcher', anonymous=True)
    bridge = CvBridge()
    
    top_image_sub = message_filters.Subscriber('/cam_top/image_eqrect_drop', Image)
    btm_image_sub = message_filters.Subscriber('/cam_bottom/image_eqrect_drop', Image)

    ts = message_filters.ApproximateTimeSynchronizer([top_image_sub, btm_image_sub], 10, 0.010)
    ts.registerCallback(callback)
    
    rospy.spin()

