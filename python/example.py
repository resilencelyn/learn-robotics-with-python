#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
************************************************************************
Test the robotics_toolbox for python
************************************************************************
Author: James CHAN
Email:  522706601@qq.com
Date:   Jan, 2020
************************************************************************
'''

import numpy as np
import math
import robotics_toolbox as rtb

def test_euler_to_matrix():
    euler = np.array([0, 0, 0])
    print("\nThe euler angles are:")
    print(euler)
    print("The corresponding rotation matrix is:")
    print(rtb.euler_to_matrix(euler))

def test_matrix_to_euler():
    matrix = np.mat([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]])
    print("\nThe rotation matrix is:")
    print(matrix)
    print("The corresponding euler angles are:")
    print(rtb.matrix_to_euler(matrix))
    
def test_pose_to_trans():
    pose = np.array([1, 2, 3, 0, math.pi / 2, 0])
    trans = rtb.pose_to_trans(pose)
    print("\nThe 6-d pose vector is:")
    print(pose)
    print("The corresponding homogeneous transformation matrix is:")
    print(trans)

if __name__ == "__main__":
    test_euler_to_matrix()
    test_matrix_to_euler()
    test_pose_to_trans()