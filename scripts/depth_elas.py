#!/usr/bin/env python
# license removed for brevity

import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image, PointField, PointCloud2
import os
import time

from cv_bridge import CvBridge, CvBridgeError
from elas import *

def xyzrgb_array_to_pointcloud2(points, colors, stamp=None, frame_id=None, seq=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    assert(points.shape == colors.shape)

    buf = []

    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if seq: 
        msg.header.seq = seq
    msg.height = points.shape[0]
    msg.width = points.shape[1]
    N = msg.width
    #xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
    xyzrgb = np.array(np.concatenate((points, colors), axis=2), dtype=np.float32)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('b', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('r', 20, PointField.FLOAT32, 1)
    ]
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * N
    msg.is_dense = True; 
    msg.data = xyzrgb.tostring()

    return msg 

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
    #subimgL = cv2.cvtColor(imgL[:,64:576], cv2.COLOR_BGR2GRAY)
    #subimgR = cv2.cvtColor(imgR[:,64:576], cv2.COLOR_BGR2GRAY)
    subimgL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    subimgR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    d1 = np.empty_like(subimgL, dtype=np.float32)
    d2 = np.empty_like(subimgR, dtype=np.float32)

    param = Elas_parameters()
    param.disp_min = 0
    param.disp_max = 112
    param.support_threshold = 0.85
    param.support_texture       = 10
    param.candidate_stepsize    = 5
    param.incon_window_size     = 5
    param.incon_threshold       = 5
    param.incon_min_support     = 5
    param.add_corners           = False
    param.grid_size             = 10
    param.beta                  = 0.02
    param.gamma                 = 3
    param.sigma                 = 1
    param.sradius               = 2
    param.match_texture         = 1
    param.lr_threshold          = 2
    param.speckle_sim_threshold = 2
    param.speckle_size          = 200
    param.ipol_gap_width        = 3
    param.filter_median         = False
    param.filter_adaptive_mean  = False
    param.postprocess_only_left = True
    param.subsampling           = False
    elas = Elas(param)

    elas.process_stereo(subimgL, subimgR, d1, d2)

    disp = rotate_ccw_90(d1)
    valid = (disp>0)

    output = disp * valid
    image_message = bridge.cv2_to_imgmsg((output/112.0*255).astype(np.uint8), encoding="mono8")
    image_message.header.stamp = top_image.header.stamp
    pub_disp.publish(image_message)

    distance = 0.35 * sinvnfs / np.sin(disp*np.pi/640.0)
    distance = np.clip(distance, 0, 10)
    distance = distance * valid + 0 * (1 - valid)

    #np.save(image_path + "depth%04d" % image_number, distance)
    #image_number += 1

    norm_dist = (distance * 255 / 10).astype(np.uint8)
    image_message = bridge.cv2_to_imgmsg(norm_dist, encoding="mono8")
    image_message.header.stamp = top_image.header.stamp
    pub_depth.publish(image_message)

    xyz = np.array(sphere, copy=True)
    xyz[:,:,0] = xyz[:,:,0] * distance
    xyz[:,:,1] = xyz[:,:,1] * distance
    xyz[:,:,2] = xyz[:,:,2] * distance
    
    pointcloud = xyzrgb_array_to_pointcloud2(xyz[112:528,:,:], btm_panorama[112:528,:,:].astype(np.float32)/255.0, stamp = top_image.header.stamp, frame_id="map")
    
    pubPCL.publish(pointcloud)

    return

if __name__ == '__main__':
    rospy.init_node('disparity_sgbm', anonymous=True)
    bridge = CvBridge()
    
    top_image_sub = message_filters.Subscriber('/top/image_rect', Image)
    btm_image_sub = message_filters.Subscriber('/bottom/image_rect', Image)

    ts = message_filters.ApproximateTimeSynchronizer([top_image_sub, btm_image_sub], 10, 0.005)
    ts.registerCallback(callback)
    
    pub_disp  = rospy.Publisher('image_disparity', Image, queue_size=10)
    pub_depth = rospy.Publisher('image_depth', Image, queue_size=10)


    sinvnfs = np.zeros((640,1280), dtype=np.float32)
    for u in range(1280):
        sinvnfs[:,u] = np.sin(np.arange(640, dtype=np.float32) * np.pi / 640)

    sphere = np.zeros((640,1280,3), dtype=np.float32)
    x = np.arange(1280, dtype=np.float32)
    lam = (x - 640) * (2*np.pi) / 1280
    for y in range(640):
        phi = (320 - y) * np.pi / 640
        xs = np.cos(phi)*np.sin(lam)
        ys = np.cos(phi)*np.cos(lam)
        zs = np.sin(phi) * np.ones(1280)
        sphere[y,:,0] = xs
        sphere[y,:,1] = ys
        sphere[y,:,2] = zs

    pubPCL = rospy.Publisher('pointcloud', PointCloud2, queue_size=10)

    rospy.spin()
