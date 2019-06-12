#!/usr/bin/env python
# license removed for brevity

import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
    btm_b_center = cv2.cvtColor(btm_panorama[:,160:480], cv2.COLOR_BGR2GRAY)
    
    orb = cv2.ORB_create()
    kpt = orb.detect(top_bf_stitch, None)
    kpt, dest = orb.compute(top_bf_stitch, kpt)
    kpb = orb.detect(btm_b_center, None)
    kpb, desb = orb.compute(btm_b_center, kpb)
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(dest, desb)
    matches = [m for m in matches if m.distance<MAX_DIST]
    matches = sorted(matches, key = lambda x:x.distance)

    img3 = cv2.drawMatches(top_bf_stitch, kpt, btm_b_center, kpb, matches, None, flags=2)

    cv2.imshow("Feature matching top bf", img3)

    top_fb_stitch = cv2.cvtColor(top_panorama[:,800:1120], cv2.COLOR_BGR2GRAY)
    btm_f_center = cv2.cvtColor(btm_panorama[:,800:1120], cv2.COLOR_BGR2GRAY)
    
    orb = cv2.ORB_create()
    kpt = orb.detect(top_fb_stitch, None)
    kpt, dest = orb.compute(top_fb_stitch, kpt)
    kpb = orb.detect(btm_f_center, None)
    kpb, desb = orb.compute(btm_f_center, kpb)
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(dest, desb)
    matches = [m for m in matches if m.distance<MAX_DIST]
    matches = sorted(matches, key = lambda x:x.distance)

    img3 = cv2.drawMatches(top_fb_stitch, kpt, btm_f_center, kpb, matches, None, flags=2)

    cv2.imshow("Feature matching top fb", img3)

    btm_bf_stitch = cv2.cvtColor(btm_panorama[:,480:800], cv2.COLOR_BGR2GRAY)
    top_f_center = cv2.cvtColor(top_panorama[:,480:800], cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create()
    kpt = orb.detect(top_f_center, None)
    kpt, dest = orb.compute(top_f_center, kpt)
    kpb = orb.detect(btm_bf_stitch, None)
    kpb, desb = orb.compute(btm_bf_stitch, kpb)
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(dest, desb)
    matches = [m for m in matches if m.distance<MAX_DIST]
    matches = sorted(matches, key = lambda x:x.distance)

    img3 = cv2.drawMatches(top_f_center, kpt, btm_bf_stitch, kpb, matches, None, flags=2)

    cv2.imshow("Feature matching bottom bf", img3)

    btm_fb_stitch = cv2.cvtColor(np.concatenate((btm_panorama[:,1120:1280],btm_panorama[:,0:160]), axis=1), cv2.COLOR_BGR2GRAY)
    top_b_center = cv2.cvtColor(np.concatenate((top_panorama[:,1120:1280],top_panorama[:,0:160]), axis=1), cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create()
    kpt = orb.detect(top_b_center, None)
    kpt, dest = orb.compute(top_b_center, kpt)
    kpb = orb.detect(btm_fb_stitch, None)
    kpb, desb = orb.compute(btm_fb_stitch, kpb)
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(dest, desb)
    matches = [m for m in matches if m.distance<MAX_DIST]
    matches = sorted(matches, key = lambda x:x.distance)

    img3 = cv2.drawMatches(top_b_center, kpt, btm_fb_stitch, kpb, matches, None, flags=2)

    cv2.imshow("Feature matching bottom fb", img3)

    cv2.waitKey(1)    


if __name__ == '__main__':
    rospy.init_node('feature_matcher', anonymous=True)
    bridge = CvBridge()
    
    top_image_sub = message_filters.Subscriber('/cam_top/image_eqrect_drop', Image)
    btm_image_sub = message_filters.Subscriber('/cam_bottom/image_eqrect_drop', Image)

    ts = message_filters.ApproximateTimeSynchronizer([top_image_sub, btm_image_sub], 10, 0.010)
    ts.registerCallback(callback)
    
    rospy.spin()

