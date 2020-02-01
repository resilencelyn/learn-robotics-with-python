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
import robotics_toolbox as rtb

def test_euler_to_matrix():
    euler = np.array([0, 0, 0])
    print("\nThe euler angles are:")
    print(euler)
    print("The corresponding rotation matrix are:")
    print(rtb.euler_to_matrix(euler))

def test_matrix_to_euler():
    matrix = np.mat([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]])
    print("\nThe rotation matrix are:")
    print(matrix)
    print("The corresponding euler angles are:")
    print(rtb.matrix_to_euler(matrix))

if __name__ == "__main__":
    test_euler_to_matrix()
    test_matrix_to_euler()