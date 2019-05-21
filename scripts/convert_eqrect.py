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

def read_params_camodocal(filename):
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

def initRectifyMap(K, D, xi):
    flags = cv2.omnidir.RECTIFY_LONGLATI
    new_size = (640, 640)
    Knew = np.eye(3)
    nh = new_size[0]
    nw = new_size[1]
    Knew[0][0] = nw / 3.141592
    Knew[1][1] = nh / 3.141592
    R = np.eye(3)
    map1, map2 = cv2.omnidir.initUndistortRectifyMap(K, D, xi, R, Knew, new_size, cv2.CV_16SC2, flags)
    return map1, map2

def equirectangular_projection(img, map1, map2):
    img_t = cv2.transpose(img)
    img_f = cv2.flip(img_t, flipCode=1)
    undistorted_img = cv2.remap(img_f, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    img_t = cv2.transpose(undistorted_img)
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

    front_eqimg = equirectangular_projection(front_img, map1_f, map2_f)
    back_eqimg  = equirectangular_projection(back_img,  map1_b, map2_b)

    #pool = Pool(processes=2)
    #front_eqimg = pool.apply_async(equirectangular_projection, (front_img, map1_f, map2_f))
    #back_eqimg  = pool.apply_async(equirectangular_projection, (back_img,  map1_b, map2_b))
    #pool.close()
    #pool.join()
    #both_eqimg = np.concatenate((front_eqimg.get(), back_eqimg.get()), axis=1)

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

    if rospy.has_param('~front_calib'):
        params_file = rospy.get_param('~front_calib')
        xi_f, K_f, D_f = read_params_camodocal(params_file)
        map1_f, map2_f = initRectifyMap(K_f, D_f, xi_f)
    else:
        print("Front camera calibration file is missing.")
        sys.exit(-1)

    if rospy.has_param('~back_calib'):
        params_file = rospy.get_param('~back_calib')
        xi_b, K_b, D_b = read_params_camodocal(params_file)
        map1_b, map2_b = initRectifyMap(K_b, D_b, xi_b)
    else:
        print("Back camera calibration file is missing.")
        sys.exit(-1)

    bridge = CvBridge()
    sub = rospy.Subscriber("image_raw", Image, callback)
    pub = rospy.Publisher('image_eqrect', Image, queue_size=10)

    rospy.spin()


