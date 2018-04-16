#!/usr/bin/env python
# -*- coding: utf-8 -*-

## will replace dataset2bag
import rosbag
import argparse
import math
import numpy as np
import cv2
#from cv2 import cv2util
import os
import yaml
import tf
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, TransformStamped
from tf2_msgs.msg import TFMessage

#################### imu functions #########################

# get parameters from the name of the file and modify imu values to correct error and mesure units
def modify_imu_data(line):
#since number of characters of the values may change lines are splitted usings spaces as delimiters	
  splitted_line = line.split(" ")
  seconds,nanoseconds = splitted_line[0].split(".")
  nanoseconds = nanoseconds + "000"
  Gx = float(splitted_line[3])
  Gy = float(splitted_line[4])
  Gz = float(splitted_line[5])
  Tx = float(splitted_line[6])
  Ty = float(splitted_line[7])
  Tz = float(splitted_line[8])

### adjustment of value's offsets (everythin in g and Hz)
  with open(args.calibration, 'r') as stream:
    try:
      data_offset = yaml.load(stream)
      acc_offset = data_offset['imu']['acc_offset']
      gyro_offset = data_offset['imu']['gyro_offset']
    except yaml.YAMLError as exc:
      print(exc) 
  Gx = Gx+float(gyro_offset[0])
  Gy = Gy+float(gyro_offset[1])
  Gz = Gz+float(gyro_offset[2])
#  Tx = float(Tx)+float(acc_offset[0])
#  Ty = float(Ty)+float(acc_offset[1])
#  Tz = float(Tz)+float(acc_offset[2])

### convertion from g to m/s²
  Tx_meters = (Tx * 9.80665)/1000
  Ty_meters = (Ty * 9.80665)/1000
  Tz_meters = (Tz * 9.80665)/1000

### convertion from degrees to rad/s
  Gx_rads = Gx * math.pi / 180
  Gy_rads = Gy * math.pi / 180
  Gz_rads = Gz * math.pi / 180

  return int(seconds), int(nanoseconds), Gx_rads, Gy_rads, Gz_rads, Tx_meters, Ty_meters, Tz_meters      

# save imu msg to the ROSBAG
def save_imu_bag(frame_id, seq, seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz):
  ros_imu = Imu()
  ros_imu.header.seq = seq
  ros_imu.header.stamp.secs = seconds
  ros_imu.header.stamp.nsecs = nanoseconds
  ros_imu.header.frame_id = frame_id
  imu_topic = "/imu"

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
  ros_imu.linear_acceleration_covariance = ros_imu.orientation_covariance = ros_imu.angular_velocity_covariance
  #for the orientation we need to put -1 in the first value of covariance to show we do not have orientation

  bag.write(imu_topic, ros_imu, ros_imu.header.stamp)

#################### images functions #########################

# read the yaml file and get the information for the camera calibration, the camera is parse as arg (cam0 =left , cam1 = right)
def get_camera_info(camera_info, camera):
  with open(camera_info, 'r') as stream:
    try:
      data = yaml.load(stream)
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
#if cam0 then it's left camera,so R = identity and T = [0 0 0]
      if camera == "cam0":
        camera_info.R[0:3] = [1, 0, 0]
        camera_info.R[3:6] = [0, 1, 0]
        camera_info.R[6:9] = [0, 0, 1]
      else:
        camera_info.R[0:3] = data[camera]['T_cn_cnm1'][0][:3]
        camera_info.R[3:6] = data[camera]['T_cn_cnm1'][1][:3]
        camera_info.R[6:9] = data[camera]['T_cn_cnm1'][2][:3]
        T[0:3] = [data[camera]['T_cn_cnm1'][0][3] ,data[camera]['T_cn_cnm1'][1][3], data[camera]['T_cn_cnm1'][2][3]]

    except yaml.YAMLError as exc:
      print(exc)

  return camera_info, T
# get the image from the path and the parameters from the name

def rectify_images(cam0,cam1,T):
  R1_rectified = np.zeros((3,3))
  R2_rectified = np.zeros((3,3))
  P1_rectified = np.zeros((3,4))
  P2_rectified = np.zeros((3,4))
  Q_rectified = np.zeros((4,4))

  cv2.stereoRectify(np.reshape(cam0.K,(3,3)),np.reshape(cam0.D,(5,1)),np.reshape(cam1.K,(3,3)), np.reshape(cam1.D,(5,1)), (cam0.width, cam0.height), np.reshape(cam1.R,(3,3)), np.reshape(T,(3,1)), R1_rectified, R2_rectified, P1_rectified, P2_rectified, Q_rectified)
  cam0.R = list(R1_rectified.flat)
  cam0.P = list(P1_rectified.flat)
  cam1.R = list(R2_rectified.flat)
  cam1.P = list(P2_rectified.flat)
  return cam0, cam1

def get_image(path, filename):
  image = cv2.imread(path + "/" + filename)
  splitted_filename = filename.split("_")
  seconds, nanoseconds, compressed_format = splitted_filename[1].split(".")
  nanoseconds = nanoseconds + "000"
  frame_id = splitted_filename[0]
  return frame_id, int(seconds), int(nanoseconds), image

# save img msg to the ROSBAG. It doesn't matter if its right or left, the difference comes with frame_ïd  
def save_image_bag(frame_id,seq, seconds, nanoseconds, image, ros_image_config):
  ros_image = Image()
  img_topic = "/stereo/" + frame_id + "/image_raw"
  img_config_topic = "/stereo/" + frame_id + "/camera_info"
  ros_image.header.frame_id = frame_id
  ros_image.header.seq = seq
  ros_image.header.stamp.secs = seconds
  ros_image.header.stamp.nsecs = nanoseconds
  ros_image.height = image.shape[0] #rows
  ros_image.width = image.shape[1] #columns
  ros_image.step = image.strides[0] 
  ros_image.encoding = "bgr8"
  ros_image.data = image.tostring()

  ros_image_config.header.frame_id = frame_id + "_camera_info"
  ros_image_config.header.stamp = ros_image.header.stamp
  ros_image_config.header.seq = seq
  bag.write(img_topic, ros_image, ros_image.header.stamp) # write image in bag
  bag.write(img_config_topic, ros_image_config, ros_image_config.header.stamp) # write calibration for the image in bag


#################### gps functions #########################

# get the gps information necesary for fix message from the line (that is presumed to be GGA)
def get_gps_data_fromGGA(line):
  timestamp = line.split(' ')[0]
  seconds = timestamp.split('.')[0]
  nanoseconds = timestamp.split('.')[1] + "000" 
  sentencesData = line.split(',')

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

  return int(seconds), int(nanoseconds), status, service, latitude, longitude, altitude, position_covariance, position_covariance_type

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
  timestamp = line.split(' ')[0]
  seconds = timestamp.split('.')[0]
  nanoseconds = timestamp.split('.')[1] + "000" 
  sentencesData = line.split(',')
  velocity_meters = float(sentencesData[7]) * 0.514444
  angle_rads =  float(sentencesData[8]) * math.pi / 180 
  v_linear_x = velocity_meters * math.sin(angle_rads)
  v_linear_y = velocity_meters * math.cos(angle_rads)

  return int(seconds), int(nanoseconds), v_linear_x, v_linear_y

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
def get_odom(line):
  sentence = line.split(" ")
  seconds, nanoseconds = sentence[0].split(".")
  nanoseconds = nanoseconds + "000"
  data = sentence[2].split(",")
  velo_1 = float(data[5])
  angle = float(data[16])
  direction = data[17][:-3]
  if direction == "0": # change direction ford = 0 back = -1 to ford = 1 back = -1
  	direction = "1"
  d = 0.57 # diameter of the wheel in meters

  velo_1_meters = velo_1 * math.pi * d * 60
  angle_rads = math.radians(angle)

  return seconds, nanoseconds, velo_1_meters, angle_rads, direction


def calculate_odom(x, y, vel, angle, angle_old , direction, delta_t):

  v_x = vel * math.sin(angle)
  v_y = vel * math.cos(angle)

  x = x + v_x * delta_t
  y = y + v_y * delta_t

  v_ang = (angle - angle_old) 

  return x, y , v_x, v_y , v_ang 

def calculate_orientation():

def save_odom_ros():



#################### TF functions #########################

def get_transformation(from_frame_id, to_frame_id, transform):
  if to_frame_id == "gps-rtk":
    t=transform['position_gps_imu']
    q=transform['orientation_gps_imu']
  else:
    t = transform[0:3, 3]
    q = tf.transformations.quaternion_from_matrix(transform)
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

def save_tf_bag(tfm, timestamps):
  seq = 0
  tf_topic = "tf"
  for timestamp in timestamps:
    for i in range(len(tfm.transforms)):
      tfm.transforms[i].header.seq = seq
      tfm.transforms[i].header.stamp = timestamp
    bag.write(tf_topic, tfm, timestamp)
    seq = seq +1
#############################################
if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description='Script that takes images imu and gps along with the calibration info to create a rosbag')
  parser.add_argument('--imu', help='imu log file')
  parser.add_argument('--images', help='folder for the images')
  parser.add_argument('--calibration', help='yaml file with the calibration')
  parser.add_argument('--gps', help='gps log file')
  parser.add_argument('--odom', help='odometry log file with speed and angle')
  args = parser.parse_args()
  bag = rosbag.Bag('dataset.bag', 'w')
################## imu part
  if args.imu:
    fr = open(args.imu,"r") #information obtained from sensor
    seq = 0
    imu_frame_id = "imu"
    global_timestamps = []
    total_size = os.path.getsize(args.imu)
    for line in fr:
      seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz = modify_imu_data(line)
      save_imu_bag(imu_frame_id, seq, seconds, nanoseconds, Gx, Gy, Gz, Tx, Ty, Tz)
      seq = seq + 1     # increment seq number
      global_timestamps.append(rospy.Time(seconds,nanoseconds))
      if seq < (total_size/73):
      	print "imu processed: " + str(seq) + "/" + str(total_size/73) +"\n"
      else:
        print "imu processed: " + str(total_size/73) + "/" + str(total_size/73) +"\n"
      ################## images part
  if args.images and args.calibration:
    i=0
    k=0
    camera_info = [0, 0]
    camera_info_rect = [0, 0]
    image_r_frame_id = "right"
    image_l_frame_id = "left"
    for i in range(0,2):
      camera_info[i],T = get_camera_info(args.calibration, "cam" + str(i))
    camera_info_rect[0],camera_info_rect[1] = rectify_images(camera_info[0], camera_info[1], T) 
    seq_right = 0
    seq_left = 0
    for filename in sorted(os.listdir(args.images)): 
      camera, seconds, nanoseconds, image = get_image(args.images, filename)
      if camera == "right":
        save_image_bag(image_r_frame_id, seq_right, seconds, nanoseconds, image, camera_info[1])      
        seq_right = seq_right + 1
        
      if camera == "left":
        save_image_bag(image_l_frame_id, seq_left, seconds, nanoseconds, image, camera_info[0])
        seq_left = seq_left + 1
      k = k+ 1
      print "images processed: " + str(k) +"/" + str(len(os.listdir(args.images))) +"\n"

################## gps part
  if args.gps:
    fr = open(args.gps,"r") #information obtained from sensor
    seq_GGA = 0
    seq_RMC = 0
    gps_frame_id = "gps-rtk"
    for line in fr:	
      if "GGA" in line:
        seconds, nanoseconds, status, service, latitude, longitude, altitude, position_covariance, position_covariance_type = get_gps_data_fromGGA(line)
        save_gps_bag(gps_frame_id, seq_GGA, seconds, nanoseconds, status, service, latitude, longitude, altitude, position_covariance, position_covariance_type)
        seq_GGA = seq_GGA + 1     # increment seq number
      
      if "RMC" in line:
        seconds, nanoseconds, v_linear_x, v_linear_y = get_gps_data_fromRMC(line)
        save_gps_RMC_bag(gps_frame_id, seq_RMC, seconds, nanoseconds, v_linear_x, v_linear_y)
        seq_RMC = seq_RMC + 1     # increment seq number

################# odometry part

  if args.odom:
    fr = open(args.odom, "r")
    seq = 0
    x = 0
    y = 0
    angle_old = 0 
    for line in fr:
  	  seconds, nanoseconds, velo_1, angle, direction = get_odom(line)
  	  delta_t = 0.1 # needs to be changed to the time diference between timestamps
  	  x, y, v_x, v_y, v_ang = calculate_odom(x, y, velo_1, angle, angle_old, direction, delta_t)
  	  #orientation = get_orientation(angle, delta_t)
  	  #save_odom_bag(seconds, nanoseconds, v_x, v_y, v_ang, x, y, orientation)
  	  angle_old = angle
  	  seq = seq + 1 

################# transformations part
  if args.calibration:

    with open(args.calibration, 'r') as stream:
      try:
        data = yaml.load(stream)
        T_base_link_to_imu = np.eye(4, 4)
        T_cam_l_to_imu = np.matrix(data['cam0']['T_cam_imu'])
        T_cam_r_to_imu = np.matrix(data['cam1']['T_cam_imu'])
        T_gps_to_imu = data['gps']

        transforms = [
        ('base_link', imu_frame_id, T_base_link_to_imu),
        (imu_frame_id, image_l_frame_id , T_cam_l_to_imu),
        (imu_frame_id, image_r_frame_id, T_cam_r_to_imu),
        (imu_frame_id, gps_frame_id, T_gps_to_imu) #it is already in position-orientation(Quat), no need for transf
        ]
        tfm = TFMessage()
        for transform in transforms:
          tf_msg = get_transformation(transform[0],transform[1], transform[2])
          tfm.transforms.append(tf_msg)
        save_tf_bag(tfm, global_timestamps)
      except yaml.YAMLError as exc:
        print(exc)
  
  bag.close()