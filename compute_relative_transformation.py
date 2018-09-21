#!/usr/bin/env python
import numpy as np
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
R1_baselink_cam = np.array([[ 0.0417161,  -0.31612375,  0.94780037],
               [-0.99910656, -0.00677043,  0.0417161 ],
               [-0.00677043, -0.94869381, -0.31612375]])

# Sequence 02
# Rotation
R2_baselink_cam = np.array([[ 0.05952187, -0.23288002,  0.97068226],
               [-0.99820222, -0.00703384,  0.05952187],
               [-0.00703384, -0.97248004, -0.23288002]])

# Sequence 03
# Rotation
R3_baselink_cam = np.array([[ 0.05593014, -0.09739749,  0.99367276],
               [-0.99843095, -0.00273237,  0.05593014],
               [-0.00273237, -0.99524181, -0.09739749]])

# Sequence 04
# Rotation
R4_baselink_cam = np.array([[ 0.1297657, -0.10171678, 0.98631362],
               [-0.99152242, -0.00664515, 0.1297657],
               [-0.00664515, -0.9947912, -0.10171678]])

# Sequence 05
# Rotation
R5_baselink_cam = np.array([[ 0.110789, -0.10328158, 0.9884628],
               [-0.99382729, -0.00575443,  0.110789],
               [-0.00575443, -0.99463551, -0.10328158]])

# Sequence 06
# Rotation
R6_baselink_cam = np.array([[ 0.07379323, -0.07838032, 0.99418866],
              [-0.99726935, -0.0029004, 0.07379323],
              [-0.0029004, -0.99691931, -0.07838032]])

# R_leftCam_imu obtained from kalibr
R_cam_imu = np.array([[0.009471441780975032, 0.9984567420628084, 0.05472134884952806],
                      [-0.9939081748565041, 0.0033909183804303744, 0.11015916496574298],
                      [0.10980360533243941, -0.055431362078334365, 0.9924064350628417]])

print ("quaternion represetend as [w,x,y,z]")
for R_baselink_cam in [R1_baselink_cam, R2_baselink_cam, R3_baselink_cam, R4_baselink_cam, R5_baselink_cam, R6_baselink_cam]:
  R_baselink_imu = R_baselink_cam * R_cam_imu
  # quaternion represetend as [w,x,y,z]
  q = tf.quaternion_from_matrix(R_baselink_imu)
  print (q)
