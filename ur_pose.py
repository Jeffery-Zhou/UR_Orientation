from __future__ import division
import urx
import time
import numpy as np
# import rospy
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


def find_orientation(path_lst, normals):
    # points = np.asarray(path_lst)
    points = [np.array(i) for i in path_lst]
    # normals = [np.array([-0.075, -0.7, 0.0]) - np.array(i) for i in path_lst]
    rotvecs = []
    for i in range(len(path_lst)-1):
        pos_diff = 0
        pos_diff = points[i+1]-points[i]
        z_dir = np.array(normals[i])

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

path_lst = []
normals = []
path_lst.append([0.884614, -0.939113, 0.416244])
normals.append([-0.962958, 0.269275, 0.0142281])

path_lst.append([0.884136, -0.939999, 0.415235])
normals.append([-0.952505, 0.297177, 0.0664836])

path_lst.append([0.886458, -0.935524, 0.414231])
normals.append([-0.978429, 0.206582, 0.000666944])

path_lst.append([0.88494, -0.940579, 0.413911])
normals.append([-0.966314, 0.255972, 0.0267653])

path_lst.append([0.885512, -0.937304, 0.413221])
normals.append([-0.967128, 0.25367,  -0.0177677])

path_lst.append([0.883993, -0.942363, 0.412899])
normals.append([-0.966025, 0.255753, 0.0372191])


print(path_lst)
print(normals)


traj_res = find_orientation(path_lst, normals)
print(traj_res)
