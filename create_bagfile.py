#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function # print() compatible with both, python2.7 and python3
## will replace dataset2bag
import sys
import math
import numpy as np
import argparse
import rosbag
import cv2
import os
import yaml
import tf
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from rosario2euroc import save_asl_format

#################### imu functions #########################

# get imu metadata (timestamp splitted in s and ns)
def get_imu_info(line):
  splitted_line = line.split(" ")
  seconds,nanoseconds = splitted_line[0].split(".")
  nanoseconds = nanoseconds + "000"
  return int(seconds), int(nanoseconds)

# get parameters from the name of the file and modify imu values to correct error and mesure units
def modify_imu_data(line):
  seconds, nanoseconds = get_imu_info(line)

  #since number of characters of the values may change lines are splitted usings spaces as delimiters
  splitted_line = line.split(" ")
  Gx = float(splitted_line[3])
  Gy = float(splitted_line[4])
  Gz = float(splitted_line[5])
  Tx = float(splitted_line[6])
  Ty = float(splitted_line[7])
  Tz = float(splitted_line[8])

### adjustment of value's offsets (everythin in g and Hz)
  with open(args.calibration, 'r') as stream:
    try:
      data_offset = yaml.load(stream, yaml.SafeLoader)
      acc_offset = data_offset['imu']['acc_offset']
      gyro_offset = data_offset['imu']['gyro_offset']
    except yaml.YAMLError as exc:
      print(exc) 
  Gx = Gx-float(gyro_offset[0])
  Gy = Gy-float(gyro_offset[1])
  Gz = Gz-float(gyro_offset[2])
  Tx = float(Tx)-float(acc_offset[0])
  Ty = float(Ty)-float(acc_offset[1])
  Tz = float(Tz)-float(acc_offset[2])

### convertion from g to m/s²
  Tx_meters = (Tx * 9.80665)/1000.0
  Ty_meters = (Ty * 9.80665)/1000.0
  Tz_meters = (Tz * 9.80665)/1000.0

### convertion from degrees to rad/s
  Gx_rads = Gx * math.pi / 180.0
  Gy_rads = Gy * math.pi / 180.0
  Gz_rads = Gz * math.pi / 180.0

  return seconds, nanoseconds, Gx_rads, Gy_rads, Gz_rads, Tx_meters, Ty_meters, Tz_meters

def get_imu_topic():
  return "/imu"

# save imu msg to the ROSBAG
def save_imu_bag(frame_id, seq, seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz):
  ros_imu = Imu()
  ros_imu.header.seq = seq
  ros_imu.header.stamp.secs = seconds
  ros_imu.header.stamp.nsecs = nanoseconds
  ros_imu.header.frame_id = frame_id
  imu_topic = get_imu_topic()

  ros_imu.angular_velocity.x=Gx
  ros_imu.angular_velocity.y=Gy
  ros_imu.angular_velocity.z=Gz
  ros_imu.linear_acceleration.x=Tx
  ros_imu.linear_acceleration.y=Ty
  ros_imu.linear_acceleration.z=Tz

  ros_imu.orientation.x = 0.0
  ros_imu.orientation.y = 0.0
  ros_imu.orientation.z = 0.0
  ros_imu.orientation.w = 1.0

  for index in range(9):
    ros_imu.angular_velocity_covariance[index] = 0.0
    ros_imu.linear_acceleration_covariance[index] = 0.0
    ros_imu.orientation_covariance[index] = 0.0

  ros_imu.orientation_covariance[0] = -1
  #for the orientation we need to put -1 in the first value of covariance to show we do not have orientation

  bag.write(imu_topic, ros_imu, ros_imu.header.stamp)

#################### images functions #########################

# read the yaml file and get the information for the camera calibration, the camera is parse as arg (cam0 = left , cam1 = right)
def get_camera_info(camera_info, camera):
  with open(camera_info, 'r') as stream:
    try:
      data = yaml.load(stream, yaml.SafeLoader)
      camera_info = CameraInfo()
      T=[0,0,0]
      camera_info.width = data[camera]['resolution'][0]
      camera_info.height = data[camera]['resolution'][1]
      if data[camera]['distortion_model'] == "radtan":
        camera_info.distortion_model = "plumb_bob"
      else:
        camera_info.distortion_model = data[camera]['distortion_model']
      
      fx,fy,cx,cy = data[camera]['intrinsics']
      camera_info.K[0:3] = [fx, 0, cx]
      camera_info.K[3:6] = [0, fy, cy]
      camera_info.K[6:9] = [0, 0, 1]
      
      k1,k2,t1,t2 = data[camera]['distortion_coeffs']
      camera_info.D = [k1,k2,t1,t2,0]
      #if cam0 then it's left camera, so R = identity and T = [0 0 0]
      if camera == "cam0":
        camera_info.R[0:3] = [1, 0, 0]
        camera_info.R[3:6] = [0, 1, 0]
        camera_info.R[6:9] = [0, 0, 1]
      else:
        camera_info.R[0:3] = data[camera]['T_cn_cnm1'][0][:3]
        camera_info.R[3:6] = data[camera]['T_cn_cnm1'][1][:3]
        camera_info.R[6:9] = data[camera]['T_cn_cnm1'][2][:3]
        T[0:3] = [data[camera]['T_cn_cnm1'][0][3], data[camera]['T_cn_cnm1'][1][3], data[camera]['T_cn_cnm1'][2][3]]

    except yaml.YAMLError as exc:
      print(exc)

  return camera_info, T

# Get rectified matrices from the rectification function form openCV
def rectify_images(cam0, cam1, T):
  R1_rectified = np.zeros((3,3))
  R2_rectified = np.zeros((3,3))
  P1_rectified = np.zeros((3,4))
  P2_rectified = np.zeros((3,4))
  Q_rectified = np.zeros((4,4))
  flags = cv2.CALIB_ZERO_DISPARITY
  alpha = 0

  cv2.stereoRectify(np.reshape(cam0.K,(3,3)), np.reshape(cam0.D,(5,1)), np.reshape(cam1.K,(3,3)), np.reshape(cam1.D,(5,1)), (cam0.width, cam0.height), np.reshape(cam1.R,(3,3)), np.reshape(T,(3,1)), R1_rectified, R2_rectified, P1_rectified, P2_rectified, Q_rectified, flags, alpha)
  cam0.R = list(R1_rectified.flat)
  cam0.P = list(P1_rectified.flat)
  cam1.R = list(R2_rectified.flat)
  cam1.P = list(P2_rectified.flat)
  return cam0, cam1

# get the image from the path and the parameters from the name
def get_image(path, filename):
  image = cv2.imread(path + "/" + filename)
  seconds, nanoseconds, frame_id = get_image_info(filename)
  return frame_id, seconds, nanoseconds, image

# get image metadata (frame id and timestamp splitted in s and ns)
def get_image_info(filename):
  splitted_filename = filename.split("_")
  seconds, nanoseconds, _ = splitted_filename[1].split(".")
  nanoseconds = nanoseconds + "000"
  frame_id = splitted_filename[0]
  return int(seconds), int(nanoseconds), frame_id

def get_image_topic(frame_id):
  return "/stereo/" + frame_id + "/image_raw"

# save img msg to the ROSBAG. It doesn't matter if its right or left, the difference comes with frame_ïd  
def save_image_bag(frame_id,seq, seconds, nanoseconds, image, ros_image_config):
  ros_image = Image()
  img_topic = get_image_topic(frame_id)
  img_config_topic = "/stereo/" + frame_id + "/camera_info"
  ros_image.header.frame_id = "/stereo/" + frame_id
  ros_image.header.seq = seq
  ros_image.header.stamp.secs = seconds
  ros_image.header.stamp.nsecs = nanoseconds
  ros_image.height = image.shape[0] #rows
  ros_image.width = image.shape[1] #columns
  ros_image.step = image.strides[0] 
  ros_image.encoding = "bgr8"
  ros_image.data = image.tostring()

  # check http://wiki.ros.org/image_pipeline/FrameConventions
  # left and right camera_info should have the same frame_id than the left optical frame
  # we DO NOT follow this convention!!!!
  ros_image_config.header.frame_id = frame_id
  ros_image_config.header.stamp = ros_image.header.stamp
  ros_image_config.header.seq = seq
  bag.write(img_topic, ros_image, ros_image.header.stamp) # write image in bag
  bag.write(img_config_topic, ros_image_config, ros_image_config.header.stamp) # write calibration for the image in bag

#################### gps functions #########################

# get gps metadata (timestamp splitted in s and ns and rest of the line)
def get_gps_info(line):
  timestamp = line.split(' ')[0]
  seconds = timestamp.split('.')[0]
  nanoseconds = timestamp.split('.')[1] + "000"
  sentencesData = line.split(',')
  return int(seconds), int(nanoseconds), sentencesData


# get the gps information necesary for fix message from the line (that is presumed to be GGA)
def get_gps_data_fromGGA(line):
  seconds, nanoseconds, sentencesData = get_gps_info(line)

  #get status and service
  gps_qual = int(sentencesData[6])
  if gps_qual == 0:
    status = NavSatStatus.STATUS_NO_FIX
  elif gps_qual == 1:
    status = NavSatStatus.STATUS_FIX
  elif gps_qual == 2:
    status = NavSatStatus.STATUS_SBAS_FIX
  elif gps_qual in (4, 5):
    status = NavSatStatus.STATUS_GBAS_FIX
  elif gps_qual == 9:
    # Support specifically for NOVATEL OEM4 recievers which report WAAS fix as 9
    # http://www.novatel.com/support/known-solutions/which-novatel-position-types-correspond-to-the-gga-quality-indicator/
    status = NavSatStatus.STATUS_SBAS_FIX
  else:
    status = NavSatStatus.STATUS_NO_FIX
  service = NavSatStatus.SERVICE_GPS	


  # get latitude in degrees
  latitudeRaw = float(sentencesData[2])
  latitudeRawDegrees = latitudeRaw // 100 # int type division
  latitudeRawMinutes = latitudeRaw % 100
  # Get the sign of the latitude. It depends if latitude is North or South
  latitudeSign = 1
  latitudeCartidnalDirection = sentencesData[3]
  if latitudeCartidnalDirection == 'S':
    latitudeSign = -1

  latitude = latitudeSign * (latitudeRawDegrees + latitudeRawMinutes / 60.0)

  # get longitude in degrees
  longitudeRaw = float( sentencesData[4] )
  longitudeRawDegrees = longitudeRaw // 100 # division entera
  longitudeRawMinutes = longitudeRaw % 100

  # Get the sign of the longitude. It depends if longitude is West or East
  longitudeSign = 1
  longitudeCartidnalDirection = sentencesData[5]

  if longitudeCartidnalDirection == 'W':
    longitudeSign = -1

  longitude = longitudeSign * (longitudeRawDegrees + longitudeRawMinutes / 60.0)

  # get altitude in meters (9 is above sea level, 11 is sea level above ellipsoide) with 0 reference at the ellipsoide
  altitude = float(sentencesData[9]) + float (sentencesData[11])

  # get covariance using Horizontal dilution of position
  hdop = float(sentencesData[8])
  position_covariance = [0,0,0,0,0,0,0,0,0]
  position_covariance[0] = hdop ** 2
  position_covariance[1] = 0.0
  position_covariance[2] = 0.0
  position_covariance[3] = 0.0
  position_covariance[4] = hdop ** 2
  position_covariance[5] = 0.0
  position_covariance[6] = 0.0
  position_covariance[7] = 0.0
  position_covariance[8] = (2 * hdop) ** 2
  position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

  return seconds, nanoseconds, status, service, latitude, longitude, altitude, position_covariance, position_covariance_type

# save the information to the rosbag
def save_gps_bag(frame_id, seq, seconds, nanoseconds, status, service, latitude, longitude, altitude, position_covariance, position_covariance_type):
  ros_gps = NavSatFix()

  ros_gps.header.seq = seq
  ros_gps.header.stamp.secs = seconds
  ros_gps.header.stamp.nsecs = nanoseconds
  ros_gps.header.frame_id = frame_id
  gps_GGA_topic = "/gps/fix"
  ros_gps.status.status = status
  ros_gps.status.service = service
  ros_gps.latitude = latitude
  ros_gps.longitude = longitude
  ros_gps.altitude = altitude
  ros_gps.position_covariance = position_covariance
  ros_gps.position_covariance_type = position_covariance_type
  bag.write(gps_GGA_topic, ros_gps, ros_gps.header.stamp)

# get the gps information necesary for vel message from the line (that is presumed to be RMC)
def get_gps_data_fromRMC(line):
  seconds, nanoseconds, sentencesData = get_gps_info(line)
  velocity_meters = float(sentencesData[7]) * 0.514444
  angle_rads =  float(sentencesData[8]) * math.pi / 180.0
  v_linear_x = velocity_meters * math.sin(angle_rads)
  v_linear_y = velocity_meters * math.cos(angle_rads)

  return seconds, nanoseconds, v_linear_x, v_linear_y

# save the information to the rosbag
def save_gps_RMC_bag(frame_id, seq, seconds, nanoseconds, v_linear_x, v_linear_y):
  ros_vel = TwistStamped()
  ros_vel.header.seq = seq
  ros_vel.header.stamp.secs = seconds
  ros_vel.header.stamp.nsecs = nanoseconds   
  ros_vel.header.frame_id = frame_id
  gps_RMC_topic = "/gps/vel"
  ros_vel.twist.linear.x = v_linear_x
  ros_vel.twist.linear.y = v_linear_y
  bag.write(gps_RMC_topic, ros_vel, ros_vel.header.stamp)

#################### Odometry functions ###################

# get odom info (timestamp splitted in s an ns, and the rest of the line)
def get_odom_info(line):
  sentence = line.split(" ")
  seconds, nanoseconds = sentence[0].split(".")
  nanoseconds = nanoseconds + "000"
  data = sentence[2].split(",")
  return int(seconds), int(nanoseconds), data

def get_odom(line, vel_lin_prev):
  seconds, nanoseconds, data = get_odom_info(line)
  vel_1 = float(data[13]) # velocity of the first wheel
  vel_2 = float(data[9])  # velocity of the second wheel
  if (vel_1<70 and vel_2<70): # filtrating noise problem, mesures much above 5km/h are skipped
    vel_lin = (vel_1 + vel_2) / 2.0
  elif (min(vel_1, vel_2) < 70):
    vel_lin = min(vel_1, vel_2)
  else:
    vel_lin = vel_lin_prev
  angle = float(data[16]) + 5.67 # Angle offset = 5.67. The robot tends to be deviated 1.07º to the left (equivalent to 5.67) when it is moving forward.
  direction = data[17][:-3]
  if direction == "0": # change direction ford = 0 back = 1 to ford = 1 back = -1
    direction = "1"
  else:
    direction = "-1"
  d = 0.57 # diameter of the wheel in meters
  vel_lin_meters = (vel_lin * math.pi * d) / 60.0 # divide by 60.0 to get it in seconds
  angle_rads = math.radians(angle*0.20) # angle is scaled, value 100 = 20º to the right
  angle_rads = angle_rads *-1 # because angle is positive to the right, but in model positive is left
  return seconds, nanoseconds, vel_lin_meters, angle_rads, int(direction), vel_lin

# Calculate the differential equations based on the Ackerman model
def calculate_odom(x, y, theta, vel, angle, delta_t, direction):
  k=0.95 #experimental value added to adequate the model
  ang_offset = 0.0	 # It was incoroporated directly on the data
  vel = vel * direction
  v_x = vel * math.cos(theta) #vel * math.cos((math.pi/2.0) - theta) 
  v_y = vel * math.sin(theta) #vel * math.sin((math.pi/2.0) - theta) 

  x_next = v_x * delta_t + x
  y_next = v_y * delta_t + y 

  theta_next = (vel/1.6) * math.tan(k*(angle+ang_offset)) * delta_t + theta
  v_ang = (theta_next - theta) / delta_t

  return x_next, y_next, theta_next, v_x, v_y , v_ang 

# Convert the orientation angle in Quaternion for ROS
def calculate_orientation(theta):
  quat = Quaternion()
  quat = tf.transformations.quaternion_from_euler(0, 0, theta) # roll, pitch, yaw
  return quat

# Save the odometry msg in the Rosbag
def save_odom_bag(seq, seconds, nanoseconds, v_x, v_y, v_ang, x, y, orientation):

  odom_msg = Odometry()
  odom_msg.header.seq = seq
  odom_msg.header.stamp.secs = seconds
  odom_msg.header.stamp.nsecs = nanoseconds   
  odom_msg.header.frame_id = "odom"
  odom_topic = "/odom"
  odom_msg.child_frame_id = "base_link"
  odom_msg.pose.pose.position.x = x
  odom_msg.pose.pose.position.y = y
  odom_msg.pose.pose.position.z = 0
  odom_msg.pose.pose.orientation.x = orientation[0]
  odom_msg.pose.pose.orientation.y = orientation[1]
  odom_msg.pose.pose.orientation.z = orientation[2]
  odom_msg.pose.pose.orientation.w = orientation[3]
  odom_msg.twist.twist.linear.x = v_x
  odom_msg.twist.twist.linear.y = v_y
  odom_msg.twist.twist.linear.z = 0
  odom_msg.twist.twist.angular.x = 0
  odom_msg.twist.twist.angular.y = 0
  odom_msg.twist.twist.angular.z = v_ang
  bag.write(odom_topic, odom_msg, odom_msg.header.stamp)   


#################### TF functions #########################

# Inverse a rotation matrix
def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = list(t_inv.flat)
    return transform_inv

# Read the transformation from the yaml file
def get_transformation(from_frame_id, to_frame_id, transform):
  if to_frame_id == "imu":
    t = transform['position_imu_baselink']
    q=transform['rotation_imu_baselink']
  elif to_frame_id == "gps":
    t=transform['position_gps_baselink']
    q=transform['orientation_gps_baselink']
  elif from_frame_id == "odom":
    t=transform['position_baselink_odom']
    q = tf.transformations.quaternion_from_euler(transform['rotation_euler'][0],transform['rotation_euler'][1], transform['rotation_euler'][2]) # roll, pitch, yaw
  else:
    transform_inv = inv(transform) # for cameras, the tf needs to be inverted.
    t = transform_inv[0:3, 3] 
    q = tf.transformations.quaternion_from_matrix(transform_inv) 

  tf_msg = TransformStamped()
  tf_msg.header.frame_id = from_frame_id
  tf_msg.child_frame_id = to_frame_id
  tf_msg.transform.translation.x = float(t[0])
  tf_msg.transform.translation.y = float(t[1])
  tf_msg.transform.translation.z = float(t[2])
  tf_msg.transform.rotation.x = float(q[0])
  tf_msg.transform.rotation.y = float(q[1])
  tf_msg.transform.rotation.z = float(q[2])
  tf_msg.transform.rotation.w = float(q[3])
  return tf_msg

#Save the TFs in the rosbag
def save_tf_bag(tfm, timestamps, x_odom, y_odom,orientation_odom):
  seq = 0
  tf_topic = "tf_static"
  for j,timestamp in enumerate(timestamps):
    for i in range(len(tfm.transforms)):
      tfm.transforms[i].header.seq = seq
      tfm.transforms[i].header.stamp = timestamp
      # the fourth transformation is the odom->base_link transform
#    tfm.transforms[4].transform.translation.x = x_odom[j]
#    tfm.transforms[4].transform.translation.y = y_odom[j]
#    tfm.transforms[4].transform.rotation.x = orientation_odom[j][0]
#    tfm.transforms[4].transform.rotation.y = orientation_odom[j][1]
#    tfm.transforms[4].transform.rotation.z = orientation_odom[j][2]
#    tfm.transforms[4].transform.rotation.w = orientation_odom[j][3]
    bag.write(tf_topic, tfm, timestamp)
    seq = seq +1

#############################################

def get_start_time(imu, images, gps, odom):
  min_timestamp = None
  if imu:
    with open(imu,"r") as f:
      line = next(f, None)
      seconds, nanoseconds = get_imu_info(line)
      min_timestamp = rospy.Time(seconds, nanoseconds)

  if images:
    image_filename = sorted(os.listdir(images))[0]
    seconds, nanoseconds, _ = get_image_info(image_filename)
    image_first_ts = rospy.Time(seconds, nanoseconds)
    if min_timestamp is None or image_first_ts < min_timestamp:
      min_timestamp = image_first_ts

  if gps:
    with open(gps,"r") as f:
      line = next(f, None)
      seconds, nanoseconds, _ = get_gps_info(line)
      gps_first_ts = rospy.Time(seconds, nanoseconds)
      if min_timestamp is None or gps_first_ts < min_timestamp:
        min_timestamp = gps_first_ts

  if odom:
    with open(odom, "r") as f:
      line = next(f, None)
      seconds, nanoseconds, _ = get_odom_info(line)
      odom_first_ts = rospy.Time(seconds, nanoseconds)
      if min_timestamp is None or odom_first_ts < min_timestamp:
        min_timestamp = odom_first_ts

  return min_timestamp

def get_imu_time_from_line(line):
  s, ns = get_imu_info(line)
  return rospy.Time(s, ns)

if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description='Script that takes images imu and gps along with the calibration info to create a rosbag')
  parser.add_argument('--imu', help='imu log file')
  parser.add_argument('--images', help='folder for the images')
  parser.add_argument('--calibration', help='yaml file with the calibration')
  parser.add_argument('--gps', help='gps log file')
  parser.add_argument('--odom', help='odometry log file with speed and angle')
  parser.add_argument('--eqdistant_imu', action='store_true', help='Make IMU timestamps equidistant.')
  parser.add_argument('--max_duration', type=float, help='Max duration of the rosbag.')
  parser.add_argument('--save_asl_format', help='Save data as ASL format.')
  parser.add_argument('--out', help='output bag file')
  args = parser.parse_args()
  bag = rosbag.Bag(args.out, 'w')

  LEFT_FRAME_ID = "left"
  RIGHT_FRAME_ID = "right"

  if args.max_duration:
    start_time = get_start_time(args.imu, args.images, args.gps, args.odom)
    max_end_time = start_time + rospy.Duration(args.max_duration)

################## imu part ##################
  if args.imu:
    fr = open(args.imu,"r") #information obtained from sensor
    if args.eqdistant_imu:
      # Convert it to a list in order to access last element
      fr = list(fr)
      start_sec, start_nsec = get_imu_info(fr[0])
      end_sec, end_nsec = get_imu_info(fr[-1])
      samples_m_1 = len(fr) - 1
      start_ts = rospy.Time(start_sec, start_nsec)
      end_ts = rospy.Time(end_sec, end_nsec)
      # Get delta/period of time (it should be equal to 1/frequency)
      dt = (end_ts - start_ts) / samples_m_1
      curr_time = start_ts
      # print(start_ts.to_sec(), end_ts.to_sec(), samples_m_1, dt.to_sec())

    seq = 0
    imu_frame_id = "imu"
    total_size = os.path.getsize(args.imu)
    for line in fr:
      seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz = modify_imu_data(line)
      if args.eqdistant_imu:
        # Ignore seconds and nanoseconds from raw file and calculate them
        # using the initial timestamp and adding dt in each iteration.
        seconds = curr_time.secs
        nanoseconds = curr_time.nsecs
        curr_time += dt

      if args.max_duration and rospy.Time(seconds, nanoseconds) > max_end_time:
        break

      save_imu_bag(imu_frame_id, seq, seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz)
      seq = seq + 1     # increment seq number
      if seq < (total_size/73):
        print("\r imu processed: " + str(seq) + "/" + str(total_size/73), end=" ")
      else:
        print("\r imu processed: " + str(total_size/73) + "/" + str(total_size/73), end=" ")
      sys.stdout.flush() # flush terminal output
    print("") # print required to keep last printed line

################## images part ##################
  if args.images and args.calibration:
    i=0
    k=0
    camera_info = [0, 0]
    camera_info_rect = [0, 0]

    for i in range(0, 2):
      camera_info[i], T = get_camera_info(args.calibration, "cam" + str(i))
    camera_info_rect[0], camera_info_rect[1] = rectify_images(camera_info[0], camera_info[1], T)
    seq_right = 0
    seq_left = 0
    for filename in sorted(os.listdir(args.images)):

      if args.max_duration:
        seconds, nanoseconds, _ = get_image_info(filename)
        if rospy.Time(seconds, nanoseconds) > max_end_time:
          continue

      camera, seconds, nanoseconds, image = get_image(args.images, filename)

      if camera == LEFT_FRAME_ID:
        save_image_bag(LEFT_FRAME_ID, seq_left, seconds, nanoseconds, image, camera_info_rect[0])
        seq_left = seq_left + 1

      if camera == RIGHT_FRAME_ID:
        save_image_bag(RIGHT_FRAME_ID, seq_right, seconds, nanoseconds, image, camera_info_rect[1])
        seq_right = seq_right + 1

      k = k + 1
      print("\r images processed: " + str(k) +"/" + str(len(os.listdir(args.images))), end=" ")
      sys.stdout.flush() # flush terminal output
    print("") # print required to keep last printed line

################## gps part ##################
  if args.gps:
    fr = open(args.gps,"r") #information obtained from sensor
    seq_GGA = 0
    seq_RMC = 0
    gps_frame_id = "gps"
    for line in fr:
      if args.max_duration:
        seconds, nanoseconds, _ = get_gps_info(line)
        if rospy.Time(seconds, nanoseconds) > max_end_time:
          break

      if "GGA" in line:
        seconds, nanoseconds, status, service, latitude, longitude, altitude, position_covariance, position_covariance_type = get_gps_data_fromGGA(line)
        save_gps_bag(gps_frame_id, seq_GGA, seconds, nanoseconds, status, service, latitude, longitude, altitude, position_covariance, position_covariance_type)
        seq_GGA = seq_GGA + 1     # increment seq number
      
      if "RMC" in line:
        seconds, nanoseconds, v_linear_x, v_linear_y = get_gps_data_fromRMC(line)
        save_gps_RMC_bag(gps_frame_id, seq_RMC, seconds, nanoseconds, v_linear_x, v_linear_y)
        seq_RMC = seq_RMC + 1     # increment seq number

################# odometry part ##################

  if args.odom:
    fr = open(args.odom, "r")
    seq = 0
    x = 0
    y = 0
    theta = 0
    x_odom = [] #used for tf, tfor transform odom to baselink
    y_odom = [] #used for tf, tfor transform odom to baselink
    orientation_odom =[] #used for tf, tfor transform odom to baselink
    orientation = Quaternion()
    odom_frame_id = "odom"
    vel_lin_prev = 0
    global_timestamps = [] # to be used as timestamps for tf 
    for line in fr:
      seconds, nanoseconds, velo_l, angle, direction, vel_lin_prev = get_odom(line,vel_lin_prev)
      if args.max_duration and rospy.Time(seconds, nanoseconds) > max_end_time:
        break
      delta_t = 0.1 # needs to be changed to the time diference between timestamps
      x_next, y_next, theta_next, v_x, v_y, v_ang = calculate_odom(x, y, theta, velo_l, angle, delta_t,direction)
      orientation = calculate_orientation(theta)
      save_odom_bag(seq, seconds, nanoseconds, v_x, v_y, v_ang, x, y, orientation)
      global_timestamps.append(rospy.Time(seconds,nanoseconds))
      x_odom.append(x) #used for tf, tfor transform odom to baselink
      y_odom.append(y) #used for tf, tfor transform odom to baselink
      orientation_odom.append(orientation)
      x = x_next
      y = y_next
      theta = theta_next 
      seq = seq + 1

# convert data to numpy arrays
   # pos_grnd = np.array( pos_grnd )

   # xy_path = []

  #x_grnd = pos_grnd[:,0]
  #y_grnd = pos_grnd[:,1]
  #z_grnd = pos_grnd[:,2]

  #xy_path.append( (x_grnd, y_grnd) )

  #labels = np.array([ "GPS-RTK" ])
  #colors = np.array( ["black"] )
  #ph.plotPaths2D( xy_path,  labels, colors)
    #plt.plot(x_odom, y_odom)
    #plt.xlim(-100,160)
    #plt.gca().set_aspect('equal', adjustable='box')
  ####################################################################
  # Show all plots
  ####################################################################

    #plt.show()

  ####################################################################
  # quit script
  ####################################################################

    #quit()

      
################# transformations part ##################

  if args.calibration:

    with open(args.calibration, 'r') as stream:
      try:
        data = yaml.load(stream, Loader=yaml.SafeLoader)
        T_imu_baselink = data['imu']
        T_cam_l_to_imu = np.matrix(data['cam0']['T_cam_imu'])
        T_cam_r_to_imu = np.matrix(data['cam1']['T_cam_imu'])
        T_gps_to_baselink = data['gps']
        T_baselink_to_odom = data['odom']

        transforms = [
        ('base_link', imu_frame_id, T_imu_baselink),
        (imu_frame_id, LEFT_FRAME_ID , T_cam_l_to_imu),
        (imu_frame_id, RIGHT_FRAME_ID, T_cam_r_to_imu),
        ('base_link', gps_frame_id, T_gps_to_baselink) #, #it is already in position-orientation(Quat), no need for transf
        #(odom_frame_id,'base_link', T_baselink_to_odom)
        ]
        tfm = TFMessage()
        for transform in transforms:
          tf_msg = get_transformation(transform[0],transform[1], transform[2])
          tfm.transforms.append(tf_msg)
        save_tf_bag(tfm, global_timestamps, x_odom, y_odom, orientation_odom)
      except yaml.YAMLError as exc:
        print(exc)

  bag.close()

  if args.save_asl_format:
    if not args.calibration:
      raise ValueError("Calibration file must be supplied")
    save_asl_format(args.out,
                    args.calibration,
                    get_image_topic(LEFT_FRAME_ID),
                    get_image_topic(RIGHT_FRAME_ID),
                    get_imu_topic(),
                    args.save_asl_format)
