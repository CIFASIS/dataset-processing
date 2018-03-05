#!/usr/bin/env python
# -*- coding: utf-8 -*-

## will replace dataset2bag
import rosbag
import argparse
import math
import numpy as np
import cv2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import os
#################### imu functions #########################
# get parameters from the name of the file and modify imu values to correct error and mesure units
def modify_imu_data(line):
#since number of characters of the values may change lines are splitted usings spaces as delimiters	
  splitted_line = line.split(" ")
  frame_id = splitted_line[1]
  seconds,nanoseconds = splitted_line[0].split(".")
  nanoseconds = nanoseconds + "000"
  Gx = splitted_line[3]
  Gy = splitted_line[4]
  Gz = splitted_line[5]
  Tx = splitted_line[6]
  Ty = splitted_line[7]
  Tz = splitted_line[8]

### adjustment of value's offsets
  Gx = float(Gx)+4.5970119258
  Gy = float(Gy)-4.78218418728
  Gz = float(Gz)-7.36174929329
  Tx = float(Tx)-926.447738516
  Ty = float(Ty)+15.4401130742
  Tz = float(Tz)+401.003545936

### convertion from g to m/s²
  Tx_meters = Tx * 9.80665
  Ty_meters = Ty * 9.80665
  Tz_meters = Tz * 9.80665

### convertion from degrees to rad/s
  Gx_rads = Gx * math.pi / 180
  Gy_rads = Gy * math.pi / 180
  Gz_rads = Gz * math.pi / 180

  return frame_id, int(seconds), int(nanoseconds), Gx_rads, Gy_rads, Gz_rads, Tx_meters, Ty_meters, Tz_meters      

# save imu msg to the ROSBAG
def save_imu_bag(frame_id, seq, seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz):
  ros_imu = Imu()
  ros_imu.header.seq = seq
  ros_imu.header.stamp.secs = seconds
  ros_imu.header.stamp.nsecs = nanoseconds
  ros_imu.header.frame_id = frame_id
  imu_topic = "/imu/data_raw"

  ros_imu.angular_velocity.x=Gx
  ros_imu.angular_velocity.y=Gy
  ros_imu.angular_velocity.z=Gz
  ros_imu.linear_acceleration.x=Tx
  ros_imu.linear_acceleration.y=Ty
  ros_imu.linear_acceleration.z=Tz

  ros_imu.orientation.x = 0
  ros_imu.orientation.y = 0
  ros_imu.orientation.z = 0
  ros_imu.orientation.w = 1

  ros_imu.angular_velocity_covariance[0] = 0.001
  ros_imu.angular_velocity_covariance[1] = 0.0
  ros_imu.angular_velocity_covariance[2] = 0.0
  ros_imu.angular_velocity_covariance[3] = 0.0
  ros_imu.angular_velocity_covariance[4] = 0.001
  ros_imu.angular_velocity_covariance[5] = 0.0
  ros_imu.angular_velocity_covariance[6] = 0.0
  ros_imu.angular_velocity_covariance[7] = 0.0
  ros_imu.angular_velocity_covariance[8] = 0.001
  ros_imu.linear_acceleration_covariance = ros_imu.orientation_covariance = ros_imu.angular_velocity_covariance;

  bag.write(imu_topic, ros_imu, ros_imu.header.stamp)

#################### images functions #########################
# get the image from the path and the parameters from the name
def get_image(path, filename):
  image = cv2.imread(path + "/" + filename)
  splitted_filename = filename.split("_")
  seconds, nanoseconds, compressed_format = splitted_filename[1].split(".")
  nanoseconds = nanoseconds + "000"
  frame_id = splitted_filename[0] + "_img"
  return frame_id, int(seconds), int(nanoseconds), image

# save img msg to the ROSBAG. It doesn't matter if its right or left, the difference comes with frame_ïd  
def save_image_bag(frame_id,seq, seconds, nanoseconds, image):
  ros_image = Image()
  img_topic = "/stereo/" + frame_id + "img_raw"
  ros_image.header.frame_id = frame_id
  ros_image.header.stamp.secs = seconds
  ros_image.header.stamp.nsecs = nanoseconds
  ros_image.height = image.shape[0] #rows
  ros_image.width = image.shape[1] #columns
  ros_image.step = image.strides[0] 
  ros_image.encoding = "bgr8"
  ros_image.data = image.tostring()

  bag.write(img_topic, ros_image, ros_image.header.stamp)

#############################################
if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description='Script that takes images imu and gps along with the calibration info to create a rosbag')
  parser.add_argument('--imu', help='imu log file')
  parser.add_argument('--cam_left', help='folder for the lef images')
  parser.add_argument('--cam_right', help='folder for the right images')
  args = parser.parse_args()
  bag = rosbag.Bag('dataset.bag', 'w')
################## imu part
  if args.imu:
    fr = open(args.imu,"r") #information obtained from sensor
    seq = 0
    for line in fr:
      frame_id, seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz = modify_imu_data(line)
      save_imu_bag(frame_id, seq, seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz)
      seq = seq + 1     # increment seq number
      
################## images part
  if args.images:
    seq_right = 0
    seq_left = 0
    for filename in os.listdir(args.cam_right):
        frame_id, seconds, nanoseconds, image = get_image(args.cam_right, filename)
        if frame_id == "right_img":
          save_image_bag(frame_id, seq_right, seconds, nanoseconds, image)
          seq_right = seq_right + 1

        if frame_id == "left_img":
          save_image_bag(frame_id, seq_right, seconds, nanoseconds, image)
          seq_left = seq_left + 1
    bag.close()

