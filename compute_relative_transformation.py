#!/usr/bin/env python
import numpy as np
import math
from utils import transformations as tf
from numpy.linalg import inv

####################################################
## Compute transformation between IMU and GPS Antena
####################################################

# IMU
#position:
#  x: -0.0641664267064
#  y: -0.0164370734761
#  z: 1.16176302976
#orientation:
#  x: 0.0769653027773
#  y: 0.925899537619
#  z: -0.303251279382
#  w: -0.211719271579

# GPS antena
#pose:
#  position:
#    x: 0.00665347479368
#    y: -0.228122377393
#    z: 0.904231349513
#  orientation:
#    x: 0.00252618701703
#    y: 0.931532664618
#    z: -0.363402061428
#    w: -0.0133960769939


# create transformation matrix

# convert quaternions ([w,x,y,z])to transformation matrix

p2= np.array([[-0.0641664267064, -0.0164370734761, 1.16176302976, 1]])
q2 = np.array([-0.211719271579,0.0769653027773,0.925899537619,-0.303251279382])

p1 = np.array([[0.00665347479368, -0.228122377393, 0.904231349513, 1]])
q1 = np.array([-0.0133960769939, 0.00252618701703, 0.931532664618, -0.363402061428])

Tw1 = tf.quaternion_matrix(q1)
Tw2 = tf.quaternion_matrix(q2)

# replace fourth column in T1 by the position
Tw1[:,3] = p1
Tw2[:,3] = p2

# compute relative transformation between Tw1 and Tw2
T12 = inv(Tw1) * Tw2

# get relative traslation
t12 = T12[:3,3]

# get relative quaternion
R12 = T12[:3,:3]
q12 = tf.quaternion_from_matrix(T12)

#print ("T12",T12)
print ("t12", t12)
print ("q12", q12)


#############################################
## Compute Rotation between IMU and base_link
#############################################

# Sequence 01
# Rotation
R1_baselink_cam = np.array([[4.25132721e-03, -3.28406911e-01, 9.44526774e-01],
                            [-9.99990705e-01, -7.17997435e-04, 4.25132721e-03],
                            [-7.17997435e-04, -9.44536069e-01, -3.28406911e-01]])

# Sequence 02
# Rotation
R2_baselink_cam = np.array([[0.03147939, -0.21534861, 0.97602972],
                            [-0.99949851, -0.00343064, 0.03147939],
                            [-0.00343064, -0.97653121, -0.21534861]])

# Sequence 03
# Rotation
R3_baselink_cam = np.array([[0.04656128, -0.11986454, 0.99169781],
                            [-0.9989115, -0.00280216, 0.04656128],
                            [-0.00280216, -0.9927863, -0.11986454]])

# Sequence 04
# Rotation
R4_baselink_cam = np.array([[ 0.05755443, -0.12123556, 0.9909538],
                            [-0.99833622, -0.00350467, 0.05755443],
                            [-0.00350467, -0.99261758, -0.12123556]])

# Sequence 05
# Rotation
R5_baselink_cam = np.array([[-1.41279525e-04, -1.15215994e-01, 9.93340453e-01],
                            [-9.99999990e-01, 8.16602141e-06, -1.41279525e-04],
                            [8.16602141e-06, -9.93340463e-01, -1.15215994e-01]])

# Sequence 06
# Rotation
R6_baselink_cam = np.array([[ 0.03013045, -0.11112505, 0.99334958],
                            [-0.99954456, -0.00167971, 0.03013045],
                            [-0.00167971, -0.99380501, -0.11112505]])

# R_leftCam_imu obtained from kalibr
R_cam_imu = np.array([[0.009471441780975032, 0.9984567420628084, 0.05472134884952806],
                      [-0.9939081748565041, 0.0033909183804303744, 0.11015916496574298],
                      [0.10980360533243941, -0.055431362078334365, 0.9924064350628417]])

print ("q_baselink_imu")
print ("quaternion represetend as [x,y,z,w]")
for R_baselink_cam in [R1_baselink_cam, R2_baselink_cam, R3_baselink_cam, R4_baselink_cam, R5_baselink_cam, R6_baselink_cam]:
  R_baselink_imu = np.dot(R_baselink_cam, R_cam_imu)
  # quaternion represetend as [w,x,y,z]
  q_unordered = tf.quaternion_from_matrix(R_baselink_imu)
  # reprensent quaternion as [x,y,z,w]
  q = np.array([q_unordered[1], q_unordered[2], q_unordered[3], q_unordered[0]])
  print (q)
