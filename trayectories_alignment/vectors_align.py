#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
from utils import plotHelpers as ph
import matplotlib.pyplot as plt
import argparse
import tf
sptam_x = []
sptam_y = []
sptam_z = []
sptam_orig_x = []
sptam_orig_y = []
sptam_orig_z = []
vectors_orig = []

if __name__ == "__main__":
  
    parser = argparse.ArgumentParser(description='Script that takes logfile with sptam mesures and compute its rotation with the z axis')
    parser.add_argument('--file', help='sptam log file with rows containing: x y z')
    args = parser.parse_args()
    fr = open(args.file,"r")
    for line in fr:
        x,y,z = line.split(" ")
        vector = np.array([float(x), float(y), float(z)])
        vectors_orig.append(vector)
        vector_norm = vector/(np.linalg.norm(vector))
        sptam_orig_x.append(float(x))
        sptam_orig_y.append(float(y))
        sptam_orig_z.append(float(z))
        sptam_x.append(vector_norm[0])
        sptam_y.append(vector_norm[1])
        sptam_z.append(vector_norm[2])
    sptam_mean = np.array([np.mean(sptam_x),np.mean(sptam_y),np.mean(sptam_z)])
    z_axis = np.array([0,0,1])
    z_axis_norm = z_axis/np.linalg.norm(z_axis)
    sptam_mean_norm = sptam_mean/np.linalg.norm(sptam_mean)
    v = np.cross(z_axis_norm,sptam_mean_norm)
    c = np.dot(z_axis_norm,sptam_mean_norm)
    v_mat = np.array([[0, -v[2], v[1]],[v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.identity(3) + v_mat + np.matmul(v_mat,v_mat)*(1-c)/(np.linalg.norm(v)**2)

    print "R that transforms sptam vector to z axis vector"
    print np.linalg.inv(R)

    xy_path = []
    zy_path = []
    lines3D = []
    new_path = []
    x2 = []
    y2 = []
    z2 = []


    xaxis, zaxis = (1, 0, 0), (0, 0, 1)

    Rx = tf.transformations.rotation_matrix((math.pi/2.0), xaxis)
    Rz = tf.transformations.rotation_matrix((math.pi/2.0), zaxis)
    Rx = np.linalg.inv(Rx)
    Rz = np.linalg.inv(Rz)

    print "prueba"
    print Rx
    print Rz
    R_prueba = np.linalg.inv(R)
    R_final = np.matmul(np.matmul(Rz[0:3,0:3],Rx[0:3,0:3]),np.linalg.inv(R))
    print "rotation from camera to odometry"
    print R_final
    

    print "-------------------additional--------"




    for i in range(len(vectors_orig)):
        new_path.append(np.matmul(R_final,vectors_orig[i]))

    new_path =np.array(new_path)

    for j in range(len(new_path)):
        x2.append(np.array(new_path[j][0]))
        z2.append(np.array(new_path[j][2]))
        y2.append(np.array(new_path[j][1]))   




    lines3D.append( (np.array(sptam_orig_x), np.array(sptam_orig_y), np.array(sptam_orig_z)) )
    lines3D.append( (np.array(x2),np.array(y2), np.array(z2)) )
    




    labels = np.array([ "S-PTAM orig", "S-PTAM rot" ])
    colors = np.array( ["black", "blue"] )

    ph.plotPaths3D( lines3D, labels, colors )

    ####################################################################
    # Show all plots
    ####################################################################

    plt.show()