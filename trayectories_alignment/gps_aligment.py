#!/usr/bin/python

import numpy as np
import argparse
import yaml
import tf

def inv(transform):
  "Invert rigid body transformation matrix"
  R = transform[0:3, 0:3]
  t = transform[0:3, 3]
  t_inv = -1 * R.T.dot(t)
  transform_inv = np.eye(4)
  transform_inv[0:3, 0:3] = R.T
  transform_inv[0:3, 3] = list(t_inv.flat)
  return transform_inv

if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description='Script that takes a trajectory and perform a rotation based on a matrix supplied')
  parser.add_argument('--tray', help='trayectorie to rotate')
  parser.add_argument('--rot', help='file with the R matrix')
  parser.add_argument('--out', help='aligned trayectorie')
  args = parser.parse_args()

  rot_file = []


  T_b_gps = np.identity(4)

  T_b_gps[0,3] = 1.80070337
  T_b_gps[1,3] = -0.02982362
  T_b_gps[2,3] = 1.59345719

  T_gps_b = inv(T_b_gps)

  f_tray = open(args.tray,"r")
  fw = open(args.out,"w+")
  frot = open (args.rot,"r")
  for line in frot:
    line = line.replace(' [ ','')
    line = line.replace(' [','')
    line = line.replace('[ ','')
    line = line.replace('[','')
    line = line.replace(']','')
    line = line.replace('. ','.')
    line = line.replace('    ',' ')
    line = line.replace('   ',' ')
    line = line.replace('  ',' ')
    if (line != "\n"):
      data = line.split(" ")
      rot_file.append(float(data[0]))
      rot_file.append(float(data[1]))
  for line in f_tray:
    data = line.split(" ")
    pos_old = np.matrix([[float(data[1])],[float(data[2])]])
    R = np.matrix([[rot_file[0], rot_file[1]],[rot_file[2], rot_file[3]]])
    pos = R*pos_old   
    pos_gps = np.identity(4)
    pos_gps[0,3] = pos[0,0]
    pos_gps[1,3] = pos[1,0]
    pos_gps[2,3] = 0


    pos_baselink = np.matmul(np.matmul(T_b_gps,pos_gps),T_gps_b)
    
    fw.write(data[0] + ' ' + str(pos_baselink[0,3])+ ' ' + str(pos_baselink[1,3])  + "\r\n") 
