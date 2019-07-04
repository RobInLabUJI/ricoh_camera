#!/usr/bin/env python
# license removed for brevity

import cv2
import numpy as np
import rospy
import message_filters
import time
import yaml

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def read_ucm_params_kalibr(filename, camid='cam0'):
    with open(filename, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    dp = data[camid]['distortion_coeffs']
    D = np.array(dp)
    pp = data[camid]['intrinsics']
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
    Knew[0][0] = nw / np.pi
    Knew[1][1] = nh / np.pi
    R = np.eye(3)
    map1, map2 = cv2.omnidir.initUndistortRectifyMap(K, D, xi, R, Knew, new_size, cv2.CV_16SC2, flags)
    return map1, map2

def toSphereRad(theta, phi):
    out = np.zeros((3, 1280),dtype=float)
    out[0,:] = np.sin(theta) * np.cos(phi)
    out[1,:] = np.sin(phi)
    out[2,:] = np.cos(theta) * np.cos(phi)
    return out

def create_spherical_proj(K, xi, D, plus_theta, zi, rho_limit):
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    mapx = np.zeros((640, 1280), dtype=np.float32)
    mapy = np.zeros((640, 1280), dtype=np.float32)
    height, width = mapx.shape
    step_theta = 2*np.pi / width
    step_phi = np.pi / height
    for i in range(height):
        j = np.array(range(width))
        theta = j * step_theta - np.pi + plus_theta
        phi = i * step_phi - np.pi/2
        d = toSphereRad(theta, phi)
        rho = np.arccos(d[2,:])
        d[2,:] += zi
        imagePoints, _ = cv2.omnidir.projectPoints(np.reshape(np.transpose(d), (1,1280,3)), rvec, tvec, K, xi, D)
        ip = np.transpose(np.reshape(imagePoints, (1280, 2)))
        ip[:,rho>rho_limit] = -1
        mapx[i,:] = ip[0,:]
        mapy[i,:] = ip[1,:]
    mask = mapx != -1
    mapx, mapy = cv2.convertMaps(mapx, mapy, cv2.CV_16SC2)
    return mapx, mapy, mask

def simpleBlend(front, back):
    #front_gray = cv2.cvtColor(front, cv2.COLOR_BGR2GRAY)
    #back_gray  = cv2.cvtColor(back,  cv2.COLOR_BGR2GRAY)
    #f_mask = front_gray>0
    #b_mask = back_gray>0
    #intersect = np.array(f_mask * b_mask, dtype=np.uint8)
    #not_intersect = 1 - intersect

    global f_mask
    global b_mask
    global intersect
    global not_intersect

    s = front + back
    hs = front/2 + back/2
    r2 = cv2.bitwise_and(hs, hs, mask = intersect)
    r1 = cv2.bitwise_and(s, s, mask = not_intersect)
    result = r1 + r2
    return result

def equirectangular_projection(img, map1, map2):
    undistorted_img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    return undistorted_img

def callback(front_imgmsg, back_imgmsg):
    try:
      front_img = bridge.imgmsg_to_cv2(front_imgmsg, "bgr8")
      back_img  = bridge.imgmsg_to_cv2(back_imgmsg,  "bgr8")
    except CvBridgeError as e:
      print(e)

    front_eqimg = equirectangular_projection(front_img, map1_f, map2_f)
    back_eqimg  = equirectangular_projection(back_img,  map1_b, map2_b)

    #both_eqimg = np.concatenate((front_eqimg, back_eqimg), axis=1)
    front_eqimg = equirectangular_projection(front_img, map1_f, map2_f)
    back_eqimg  = equirectangular_projection(back_img,  map1_b, map2_b)
    both_eqimg = simpleBlend(front_eqimg, back_eqimg)

    #both_eqimg = np.roll(both_eqimg, ROLL, axis=1)

    #if FLIP:
    #    both_eqimg = cv2.flip(both_eqimg, flipCode=-1)
    
    image_message = bridge.cv2_to_imgmsg(both_eqimg, encoding="bgr8")
    image_message.header.stamp = front_imgmsg.header.stamp
    pub.publish(image_message)

if __name__ == '__main__':
    rospy.init_node('project_eqrect', anonymous=True)

    #if rospy.has_param('~flip'):
    #    FLIP = rospy.get_param('~flip')
    #else:
    #    FLIP = False
        
    #if rospy.has_param('~roll'):
    #    ROLL = rospy.get_param('~roll')
    #else:
    #    ROLL = 320
        
    if rospy.has_param('~camchain'):
        camchain_file = rospy.get_param('~camchain')
        if rospy.has_param('~front_cam_id'):
            fcid = rospy.get_param('~front_cam_id')
            xi_f, K_f, D_f = read_ucm_params_kalibr(camchain_file, fcid)
        else:
            print("Front camera id is missing.")
            sys.exit(-1)
        if rospy.has_param('~back_cam_id'):
            bcid = rospy.get_param('~back_cam_id')
            xi_b, K_b, D_b = read_ucm_params_kalibr(camchain_file, bcid)
        else:
            print("Back camera id is missing.")
            sys.exit(-1)        
    else:
        if rospy.has_param('~front_calib'):
            params_file = rospy.get_param('~front_calib')
            xi_f, K_f, D_f = read_ucm_params_kalibr(params_file)
        else:
            print("Front camera calibration file is missing.")
            sys.exit(-1)

        if rospy.has_param('~back_calib'):
            params_file = rospy.get_param('~back_calib')
            xi_b, K_b, D_b = read_ucm_params_kalibr(params_file)
        else:
            print("Back camera calibration file is missing.")
            sys.exit(-1)

    #map1_f, map2_f = initRectifyMap(K_f, D_f, xi_f)
    #map1_b, map2_b = initRectifyMap(K_b, D_b, xi_b)

    rho_limit = np.pi/2 * 95.0/90
    map1_f, map2_f, f_mask = create_spherical_proj(K_f, xi_f, D_f, 0, 0, rho_limit)
    map1_b, map2_b, b_mask = create_spherical_proj(K_b, xi_b, D_b, np.pi, 0.0, rho_limit)
    intersect = np.array(f_mask * b_mask, dtype=np.uint8)
    not_intersect = 1 - intersect

    bridge = CvBridge()

    front_image_sub = message_filters.Subscriber('front/image_raw', Image)
    back_image_sub  = message_filters.Subscriber('back/image_raw',  Image)

    ts = message_filters.ApproximateTimeSynchronizer([front_image_sub, back_image_sub], 10, 0.005)
    ts.registerCallback(callback)

    pub = rospy.Publisher('image_rect', Image, queue_size=10)

    rospy.spin()
