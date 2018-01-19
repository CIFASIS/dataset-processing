#!/usr/bin/python2
import os
import math
import argparse
import numpy as np
import tf
import rospy
import rosbag

if __name__ == "__main__":

  parser = argparse.ArgumentParser(description='Script that replace camera_info mesasages in a rosbag')
  parser.add_argument('bagfile', help='rosbag file to modify')
  parser.add_argument('left_camera_info', help='left_camera_info calibration file')
  parser.add_argument('right_camera_info', help='right_camera_info calibration file')

  args = parser.parse_args()
  
  KzedL = [349.201044, 0.000000, 343.900402, 0.000000, 349.029570, 188.274533, 0.000000, 0.000000, 1.000000]
  DzedL = [-0.175053, 0.026758, -0.000369, 0.000740, 0.000000]
  RzedL = [0.999876, 0.000981, 0.015721, -0.000959, 0.999999, -0.001407, -0.015722, 0.001392, 0.999875]
  PzedL = [348.460335, 0.000000, 340.026226, 0.000000, 0.000000, 348.460335, 197.984753, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]

  KzedR = [350.211457, 0.000000, 339.379579, 0.000000, 350.085655, 205.488061, 0.000000, 0.000000, 1.000000]
  DzedR = [-0.173801, 0.027385, -0.000046, 0.000624, 0.000000]
  RzedR = [0.999999, -0.000271, 0.001005, 0.000270, 0.999999, 0.001400, -0.001005, -0.001400, 0.999999]
  PzedR = [348.460335, 0.000000, 340.026226, -129.049346, 0.000000, 348.460335, 197.984753, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]

#  left_camera_info = '/home/martin/src/zed_vga_left.yaml'
#  right_camera_info = '/home/martin/src/zed_vga_right.yaml'

  try:

    # create output.bag file
    outbag = rosbag.Bag('output.bag', 'w')

    for topic, msg, t in rosbag.Bag(args.bagfile, 'r').read_messages():

      print('topic: ', topic)

      # set seq for left camera_info message
      if topic == "/stereo/left/camera_info":

            msg.K=KzedL
            msg.D=DzedL
            msg.R=RzedL
            msg.P=PzedL
            print (msg)
      if topic == "/stereo/right/camera_info":
            msg.K=KzedR
            msg.D=DzedR
            msg.R=RzedR
            msg.P=PzedR
            print (msg)

      outbag.write(topic,msg,t)
    outbag.close()		
            
  except Exception, e:

    import traceback
    traceback.print_exc() 
