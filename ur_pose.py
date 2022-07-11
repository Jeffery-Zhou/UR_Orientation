from __future__ import division
import urx
import time
import numpy as np
import rospy
import math3d as m3d
from scipy.spatial.transform import Rotation as R
np.set_printoptions(suppress=True)


def generate_left_points(r):
    theta = np.arange(5/6*np.pi, 3.2/6*np.pi, -0.01)
    x = -0.075 + r * np.cos(theta)
    z = 0.0 + r * np.sin(theta)
    y = [-0.7 for i in theta]
    p_lst = zip(x, y, z)
    return p_lst

def generate_left_back_points(r):
    theta = np.arange(1.5/6*np.pi, 3.1/6*np.pi, 0.01)
    x = -0.075 + r * np.cos(theta)
    z = 0.0 + r * np.sin(theta)
    y = [-0.7 for i in theta]
    p_lst = zip(x, y, z)
    return p_lst


def find_orientation(path_lst):
    # points = np.asarray(path_lst)
    points = [np.array(i) for i in path_lst]
    normals = [np.array([-0.075, -0.7, 0.0]) - np.array(i) for i in path_lst]
    rotvecs = []
    for i in range(len(path_lst)-1):
        pos_diff = 0
        pos_diff = points[i+1]-points[i]
        z_dir = normals[i]

        proj = np.dot(pos_diff, z_dir)*z_dir
        x_dir = pos_diff - proj
        x_dir = x_dir/np.linalg.norm(x_dir, axis=0)
        y_dir = np.cross(z_dir, x_dir)
        y_dir = y_dir/np.linalg.norm(y_dir, axis=0)
        r = R.from_dcm(np.vstack((x_dir, y_dir, z_dir)).T)
        orientation = r.as_quat()  # in the manner of
        rotvec = r.as_rotvec()
        rotvecs.append(rotvec)
    ur_poses = np.hstack((points[:-1], np.array(rotvecs)))
    return ur_poses
