#!/usr/bin/env python
import numpy as np
from utils import transformations as tf
from numpy.linalg import inv



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


