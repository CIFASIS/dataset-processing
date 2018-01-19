#!/usr/bin/python2
import os
import math
import argparse
import numpy as np
import tf
import rospy
import rosbag
import yaml


def parseCameraInfo( cameraInfo ):

  with open(cameraInfo, 'r') as stream:
      try:
          data = yaml.load(stream)
          print(data)
          cameraName = data['camera_name']
          imageWidth = data['image_width']
          imageHeight = data['image_height']
          distortionModel = data['distortion_model']
          K = data['camera_matrix']['data']
          D = data['distortion_coefficients']['data']
          R = data['rectification_matrix']['data']
          P = data['projection_matrix']['data']

      except yaml.YAMLError as exc:
          print(exc)

  return imageWidth, imageHeight, cameraName, K, distortionModel, D, R, P



if __name__ == "__main__":

  parser = argparse.ArgumentParser(description='Script that replace camera_info mesasages in a rosbag')
  parser.add_argument('bagfile', help='rosbag file to modify')
  parser.add_argument('left_camera_info', help='left_camera_info.yaml calibration file')
  parser.add_argument('right_camera_info', help='right_camera_info.yaml calibration file')

  args = parser.parse_args()
  
  # read left_camera_info.yaml and right_camera_info.yaml files
  imageWidthLeft, imageHeightLeft, cameraNameLeft, KLeft, distortionModelLeft, DLeft, RLeft, PLeft = parseCameraInfo(args.left_camera_info)
  imageWidthRight, imageHeightRight, cameraNameRight, KRight, distortionModelRight, DRight, RRight, PRight = parseCameraInfo(args.right_camera_info)


  print(imageWidthLeft)
  print(imageHeightLeft)
  print(cameraNameLeft)
  print(distortionModelLeft)

  print (KLeft)
  print (DLeft)
  print (RLeft)
  print (PLeft)

  print(imageWidthRight)
  print(imageHeightRight)
  print(cameraNameRight)
  print(distortionModelRight)

  print (KRight)
  print (DRight)
  print (RRight)
  print (PRight)

  try:

    # create output.bag file
    outbag = rosbag.Bag('output.bag', 'w')

    for topic, msg, t in rosbag.Bag(args.bagfile, 'r').read_messages():

#      print('topic: ', topic)

      # set seq for left camera_info message
      if topic == "/stereo/left/camera_info":

            msg.K=KLeft
            msg.D=DLeft
            msg.R=RLeft
            msg.P=PLeft
#            print (msg)
      if topic == "/stereo/right/camera_info":
            msg.K=KRight
            msg.D=DRight
            msg.R=RRight
            msg.P=PRight
#            print (msg)

      outbag.write(topic,msg,t)
    outbag.close()		
            
  except Exception, e:

    import traceback
    traceback.print_exc() 
