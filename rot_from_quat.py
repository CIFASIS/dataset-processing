#!/usr/bin/python

import numpy as np
#import tf
"""
q = [0, 0, 0, 1]
print quaternion_matrix
T_b_gps = tf.transformations.quaternion_matrix(q)
print T_b_gps
"""

def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = list(t_inv.flat)
    return transform_inv

T_b_gps = np.identity(4)

T_b_gps[0,3] = 1.80070337
T_b_gps[1,3] = -0.02982362
T_b_gps[2,3] = 1.59345719

T_gps_b = inv(T_b_gps)
print T_b_gps 
print T_gps_b 
