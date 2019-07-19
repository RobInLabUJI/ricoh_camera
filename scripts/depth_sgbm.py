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

def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg

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
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1)
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

    global sinvnfs
    global image_number
    global image_path

    try:
      top_panorama = bridge.imgmsg_to_cv2(top_image, "rgb8")
      btm_panorama = bridge.imgmsg_to_cv2(btm_image, "rgb8")
    except CvBridgeError as e:
      print(e)
    
    #dx = int(320 * 88.35 / 90.0)
    dx = 325
    btm_panorama = np.roll(btm_panorama, dx, axis=1)

    imgL = rotate_cw_90(btm_panorama)
    imgR = rotate_cw_90(top_panorama)
    #imgR = rotate_ccw_90(btm_panorama)
    #imgL = rotate_ccw_90(top_panorama)
    
    window_size = 3
    min_disp = 0
    num_disp = 112-min_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 7,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = -1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    #output = rotate_ccw_90((disp-min_disp)/num_disp)
    disp = np.roll(rotate_ccw_90(disp), -dx, axis=1)

    #output = rotate_cw_90((disp-min_disp)/num_disp)
    valid = (disp>0)

    output = disp * valid
    output = ((output-min_disp)/num_disp)
    
    image_message = bridge.cv2_to_imgmsg((output*255).astype(np.uint8), encoding="mono8")
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
    image_number = 0
    image_path = "/home/ricoh/Desktop/images/"

    rospy.init_node('depth_sgbm', anonymous=True)
    bridge = CvBridge()
    
    top_image_sub = message_filters.Subscriber('/top/image_rect', Image)
    btm_image_sub = message_filters.Subscriber('/bottom/image_rect', Image)

    ts = message_filters.ApproximateTimeSynchronizer([top_image_sub, btm_image_sub], 10, 0.05)
    ts.registerCallback(callback)
    
    pub_disp = rospy.Publisher('image_disparity', Image, queue_size=10)
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

    # sphere = np.zeros((640,1280,3), dtype=np.float32)
    # x = np.arange(1280, dtype=np.float32)
    # th = (x - 640) * (2*np.pi) / 1280
    # for y in range(640):
    #     phi = (320 - y) * np.pi / 640
    #     xs = np.cos(phi) * np.sin(th)
    #     ys = np.sin(phi) * np.ones(1280)
    #     zs = np.cos(phi) * np.cos(th)
    #     sphere[y,:,0] = xs
    #     sphere[y,:,1] = ys
    #     sphere[y,:,2] = zs

    pubPCL = rospy.Publisher('pointcloud', PointCloud2, queue_size=10)

    rospy.spin()
