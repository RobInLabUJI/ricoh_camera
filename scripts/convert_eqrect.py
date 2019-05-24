#!/usr/bin/env python
# license removed for brevity

import cv2
import numpy as np
import rospy
import time
import yaml

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#from multiprocessing import Pool

def read_ucm_params_camodocal(filename):
    with open(filename, 'r') as stream:
        skip_lines = 2
        for i in range(skip_lines):
            _ = stream.readline()
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    dp = data['distortion_parameters']
    D = np.array([dp['k1'], dp['k2'], dp['p1'], dp['p2']])
    pp = data['projection_parameters']
    K = np.eye(3)
    K[0][0] = pp['gamma1']
    K[1][1] = pp['gamma2']
    K[0][2] = pp['u0']
    K[1][2] = pp['v0']
    xi = np.array([data['mirror_parameters']['xi']])
    return xi, K, D

def read_ucm_params_kalibr(filename):
    with open(filename, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    dp = data['cam0']['distortion_coeffs']
    D = np.array(dp)
    pp = data['cam0']['intrinsics']
    K = np.eye(3)
    K[0][0] = pp[1]
    K[1][1] = pp[2]
    K[0][2] = pp[3]
    K[1][2] = pp[4]
    xi = np.array(pp[:1])
    return xi, K, D

def initRectifyMap(K, D, xi):
    flags = cv2.omnidir.RECTIFY_LONGLATI
    new_size = (640, 640)
    Knew = np.eye(3)
    nh = new_size[0]
    nw = new_size[1]
    Knew[0][0] = nw / 3.141592
    Knew[1][1] = nh / 3.141592
    theta = np.radians(90)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c,-s, 0), (s, c, 0), (0, 0, 1)))
    map1, map2 = cv2.omnidir.initUndistortRectifyMap(K, D, xi, R, Knew, new_size, cv2.CV_16SC2, flags)
    return map1, map2

def equirectangular_projection(img, map1, map2):
    undistorted_img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    return undistorted_img

def rotate_cw_90(img):
    img_t = cv2.transpose(img)
    img_f = cv2.flip(img_t, flipCode=0)
    return img_f

def callback(data):
    try:
      image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    (rows,cols,channels) = image.shape
    image_size = rows
    front_img = image[0:image_size, 0:image_size]
    back_img  = image[0:image_size, image_size:2*image_size]

    front_eqimg = rotate_cw_90(equirectangular_projection(front_img, map1_f, map2_f))
    back_eqimg  = rotate_cw_90(equirectangular_projection(back_img,  map1_b, map2_b))

    crop = 0
    if crop > 0:
        both_eqimg = np.concatenate((front_eqimg[:,crop:-crop,:], back_eqimg[:,crop:-crop,:]), axis=1)
    else:
        both_eqimg = np.concatenate((front_eqimg, back_eqimg), axis=1)

    panorama = np.roll(both_eqimg, 320, axis=1)

    image_message = bridge.cv2_to_imgmsg(panorama, encoding="bgr8")
    stamp = rospy.Time.from_sec(time.time()) 
    image_message.header.stamp = stamp
    pub.publish(image_message)
    cv2.imshow("Panorama", panorama)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('convert_eqrect', anonymous=True)

    if rospy.has_param('~calib_format'):
        calib_format = rospy.get_param('~calib_format')
        options = ['camodocal', 'kalibr']
        if not calib_format in options:
            print("Error: unknown calibration format, calid options are %s" % options)
            sys.exit(-1)
    else:
        calib_format = 'camodocal'

    if rospy.has_param('~front_calib'):
        params_file = rospy.get_param('~front_calib')
        if calib_format == 'camodocal':
            xi_f, K_f, D_f = read_ucm_params_camodocal(params_file)
        elif calib_format == 'kalibr':
            xi_f, K_f, D_f = read_ucm_params_kalibr(params_file)
        map1_f, map2_f = initRectifyMap(K_f, D_f, xi_f)
    else:
        print("Front camera calibration file is missing.")
        sys.exit(-1)

    if rospy.has_param('~back_calib'):
        params_file = rospy.get_param('~back_calib')
        if calib_format == 'camodocal':
            xi_b, K_b, D_b = read_ucm_params_camodocal(params_file)
        elif calib_format == 'kalibr':
            xi_b, K_b, D_b = read_ucm_params_kalibr(params_file)
        map1_b, map2_b = initRectifyMap(K_b, D_b, xi_b)
    else:
        print("Back camera calibration file is missing.")
        sys.exit(-1)

    bridge = CvBridge()
    sub = rospy.Subscriber("image_raw", Image, callback)
    pub = rospy.Publisher('image_eqrect', Image, queue_size=10)

    rospy.spin()


