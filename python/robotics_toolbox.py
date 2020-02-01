#!/usr/bin/python
# -*- coding: UTF-8 -*-

'''
************************************************************************
Some mathematical tools for Robotics
************************************************************************
Author: James CHAN
Email:  522706601@qq.com
Date:   Jan, 2020
************************************************************************
'''

import numpy as np
import math

def euler_to_matrix(euler):
    '''Converts Euler angles to rotation matrix.
    :param euler: The Euler angles
    Example Input:
        euler = np.array([0, 0, 0])
    Output:
        np.mat([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
    '''
    mat_x = np.mat([[ 1, 0,                   0                 ],
                    [ 0, math.cos(euler[0]), -math.sin(euler[0])],
                    [ 0, math.sin(euler[0]),  math.cos(euler[0])]])

    mat_y = np.mat([[ math.cos(euler[1]), 0, math.sin(euler[1])],
                    [ 0,                  1, 0                 ],
                    [-math.sin(euler[1]), 0, math.cos(euler[1])]])
    
    mat_z = np.mat([[ math.cos(euler[2]), -math.sin(euler[2]), 0],
                    [ math.sin(euler[2]),  math.cos(euler[2]), 0],
                    [ 0,                   0,                  1]])
    return mat_z * mat_y * mat_x
    
def matrix_to_euler(matrix):
    '''Converts rotation matrix to Euler angles.
    :matrix: Rotation matrix
    Example Input:
        matrix = np.mat([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])
    Output:
        np.array([0, 0, 0])
    '''
    R = matrix
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = (sy < 1e-6)
    if (not singular):
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def vec_to_so3(vec):
    '''Converts a 3-vector to an so(3) representation

    :param vec: A 3-vector
    :return: The skew symmetric representation of vec

    Example Input:
        vec = np.array([1, 2, 3])
    Output:
        np.mat([[ 0, -3,  2],
                [ 3,  0, -1],
                [-2,  1,  0]])
    '''
    return np.mat([[ 0,     -vec[2],  vec[1]],
                   [ vec[2],      0, -vec[0]],
                   [-vec[1], vec[0],  0     ]])