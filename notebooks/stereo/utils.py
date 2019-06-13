import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml

def plot(img):
    new_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    plt.imshow(new_img);

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
    Knew[0][0] = nw / 3.141592
    Knew[1][1] = nh / 3.141592
    theta = np.radians(90)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c,-s, 0), (s, c, 0), (0, 0, 1)))
    map1, map2 = cv2.omnidir.initUndistortRectifyMap(K, D, xi, R, Knew, new_size, cv2.CV_32FC1, flags)
    #map1, map2 = cv2.omnidir.initUndistortRectifyMap(K, D, xi, R, Knew, new_size, cv2.CV_16SC2, flags)
    return map1, map2

def equirectangular_projection(img, map1, map2):
    undistorted_img = rotate_ccw_90(cv2.remap(img, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT))
    return undistorted_img

def rotate_cw_90(img):
    img_t = cv2.transpose(img)
    img_f = cv2.flip(img_t, flipCode=1)
    return img_f

def rotate_ccw_90(img):
    img_t = cv2.transpose(img)
    img_f = cv2.flip(img_t, flipCode=0)
    return img_f

def rotate_180(img):
    img_f = cv2.flip(img, flipCode=-1)
    return img_f
