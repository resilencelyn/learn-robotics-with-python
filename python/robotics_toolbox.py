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

'''
*** BASIC MATHEMATICAL FUNCTIONS ***
'''

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

def near_zero(x):
    '''Determines whether a scalar is small enough to be treated as zero

    :param x: A scalar input to check
    :return: True if z is close to zero, false otherwise

    Example Input:
        x = -1e-7
    Output:
        True
    '''
    return abs(x) < 1e-6

def normalize(vec):
    '''Normalizes a vector

    :param vec: A vector
    :return: A unit vector pointing in the same direction as z

    Example Input:
        vec = np.array([1, 2, 3])
    Output:
        np.array([0.26726124, 0.53452248, 0.80178373])
    '''
    return vec / np.linalg.norm(vec)






'''
*** SCREW THEORY FUNCTIONS ***
'''

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
    
def so3_to_vec(so3):
    '''Converts an so(3) representation to a 3-vector

    :param so3: A so3 matrix
    :return: The corresponding 3-vector

    Example Input:
        so3mat = np.mat([[ 0, -3,  2],
                         [ 3,  0, -1],
                         [-2,  1,  0]])
    Output:
        np.array([1, 2, 3])
    '''
    return np.array([so3[2,1], so3[0,2], so3[1,0]])

def axis_ang_3(expc3):
    '''Converts a 3-vector of exponential coordinates for rotation into
    axis-angle form

    :param expc3: A 3-vector of exponential coordinates for rotation
    :return omghat: A unit rotation axis
    :return theta: The corresponding rotation angle

    Example Input:
        expc3 = np.array([1, 2, 3])
    Output:
        (np.array([0.26726124, 0.53452248, 0.80178373]), 3.7416573867739413)
    '''
    return (normalize(expc3), np.linalg.norm(expc3))

def matrix_exp_3(so3mat):
    '''Computes the matrix exponential of a matrix in so(3)

    :param so3mat: A 3x3 skew-symmetric matrix
    :return: The matrix exponential of so3mat

    Example Input:
        so3mat = np.array([[ 0, -3,  2],
                           [ 3,  0, -1],
                           [-2,  1,  0]])
    Output:
        np.array([[-0.69492056,  0.71352099,  0.08929286],
                  [-0.19200697, -0.30378504,  0.93319235],
                  [ 0.69297817,  0.6313497 ,  0.34810748]])
    '''
    omgtheta = so3_to_vec(so3mat)
    if near_zero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = axis_ang_3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
               + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)

def matrix_log_3(R):
    '''Computes the matrix logarithm of a rotation matrix

    :param R: A 3x3 rotation matrix
    :return: The matrix logarithm of R

    Example Input:
        R = np.array([[0, 0, 1],
                      [1, 0, 0],
                      [0, 1, 0]])
    Output:
        np.array([[          0, -1.20919958,  1.20919958],
                  [ 1.20919958,           0, -1.20919958],
                  [-1.20919958,  1.20919958,           0]])
    '''
    acosinput = (np.trace(R) - 1) / 2.0
    if acosinput >= 1:
        return np.zeros((3, 3))
    elif acosinput <= -1:
        if not near_zero(1 + R[2][2]):
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not near_zero(1 + R[1][1]):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return vec_to_so3(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)

def Rp_to_trans(R, p):
    """Converts a rotation matrix and a position vector into homogeneous
    transformation matrix

    :param R: A 3x3 rotation matrix
    :param p: A 3-vector
    :return: A homogeneous transformation matrix corresponding to the inputs

    Example Input:
        R = np.mat([[1, 0,  0],
                    [0, 0, -1],
                    [0, 1,  0]])
        p = np.mat([[1], 
                    [2], 
                    [5]])
    Output:
        np.mat([[1, 0,  0, 1],
                [0, 0, -1, 2],
                [0, 1,  0, 5],
                [0, 0,  0, 1]])
    """
    return np.mat(np.r_[np.c_[np.array(R), np.array([p[0,0], p[1,0], p[2,0]])], [[0, 0, 0, 1]]])

def trans_to_Rp(trans):
    """Converts a homogeneous transformation matrix into a rotation matrix
    and position vector

    :param trans: A homogeneous transformation matrix
    :return R: The corresponding rotation matrix,
    :return p: The corresponding position vector.

    Example Input:
        trans = np.mat([[1, 0,  0, 0],
                        [0, 0, -1, 0],
                        [0, 1,  0, 3],
                        [0, 0,  0, 1]])
    Output:
        (np.mat([[1, 0,  0],
                 [0, 0, -1],
                 [0, 1,  0]]),
         np.mat([[0], 
                 [0], 
                 [3]]))
    """
    trans = np.array(trans)
    return np.mat(trans[0: 3, 0: 3]), np.mat(trans[0: 3, 3]).T
