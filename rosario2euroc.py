#!/usr/bin/env python2
# -*- coding: utf-8 -*-




from __future__ import print_function
import sys
import argparse
import rosbag
import cv2
import csv
import os
from cv_bridge import CvBridge, CvBridgeError
import yaml
import numpy as np

def main():
	'''
	Save Rosario dataset files as ASL format.

	Command line arguments:
		rosbag: Rosbag file
		--imu: IMU topic
		--left: Left image topic
		--right: Right image topic
		--calib: Calibration file (yaml format)
		--output: Output folder. It must not exist, otherwise an exception is thrown.
	'''
	parser = argparse.ArgumentParser(description='Script that transform rosbag to euroc data format.')
	parser.add_argument('rosbag', help = 'rosbag file.')
	parser.add_argument("--imu", default="/imu", help="IMU topic.")
	parser.add_argument("--left", default="/stereo/left/image_raw", help="Left image topic.")
	parser.add_argument("--right", default="/stereo/right/image_raw", help="Right image topic.")
	parser.add_argument("--calib", help="Path to Calibration file.")
	parser.add_argument("--output", default="output", help="Output path")

	args = parser.parse_args()
	print(args)
	save_asl_format(args.rosbag, args.calib, args.left, args.right, args.imu, args.output)


def save_asl_format(inbag_path, calib, left_topic, right_topic, imu_topic, output):
	inbag = rosbag.Bag(inbag_path)
	bridge = CvBridge()

	parsed_calib = None
	if calib is not None:
		with open(calib) as input_calib:
			parsed_calib = yaml.load(input_calib, yaml.SafeLoader)

	if os.path.exists(output):
		raise ValueError("Output folder already exists")

	os.mkdir(output)

	path = os.path.join(output, "imu0")
	header = ["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]","a_RS_S_z [m s^-2]"]
	os.mkdir(path)
	if parsed_calib is not None:
		imu = get_imu_data(parsed_calib['imu'])
		write_imu_sensor_yaml(imu, os.path.join(path, 'sensor.yaml'))
	filepath = os.path.join(path, 'data.csv')
	with open(filepath, 'w') as csvfile:
		data_writer = csv.writer(csvfile, delimiter=',')
		data_writer.writerow(header)
		for topic, msg, t in inbag.read_messages(topics=[imu_topic]):
			data = [msg.header.stamp.to_nsec(), msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
			data_writer.writerow(data)

	path = os.path.join(output, "cam0")
	header = ["#timestamp [ns]", "filename"]
	os.mkdir(path)
	if parsed_calib is not None:
		cam0 = get_cam_data(parsed_calib['cam0'])
		write_cam_sensor_yaml(cam0, os.path.join(path, 'sensor.yaml'))
	datapath = os.path.join(path, 'data')
	os.mkdir(datapath)
	filepath = os.path.join(path, 'data.csv')
	with open(filepath, 'w') as csvfile:
		data_writer = csv.writer(csvfile, delimiter=',')
		data_writer.writerow(header)
		for topic, msg, t in inbag.read_messages(topics=[left_topic]):
			data = [str(msg.header.stamp.to_nsec()), str(msg.header.stamp.to_nsec()) + '.png']
			data_writer.writerow(data)
			cv2_img = bridge.imgmsg_to_cv2(msg, "mono8")
			imagepath = os.path.join(datapath, str(msg.header.stamp.to_nsec()) + '.png')
			cv2.imwrite(imagepath, cv2_img)

	path = os.path.join(output, "cam1")
	header = ["#timestamp [ns]", "filename"]
	os.mkdir(path)
	if parsed_calib is not None:
		cam1 = get_cam_data(parsed_calib['cam1'])
		write_cam_sensor_yaml(cam1, os.path.join(path, 'sensor.yaml'))
	datapath = os.path.join(path, 'data')
	os.mkdir(datapath)
	filepath = os.path.join(path, 'data.csv')
	with open(filepath, 'w') as csvfile:
		data_writer = csv.writer(csvfile, delimiter=',')
		data_writer.writerow(header)
		for topic, msg, t in inbag.read_messages(topics=[right_topic]):
			data = [str(msg.header.stamp.to_nsec()), str(msg.header.stamp.to_nsec()) + '.png']
			data_writer.writerow(data)
			cv2_img = bridge.imgmsg_to_cv2(msg, "mono8")
			imagepath = os.path.join(datapath, str(msg.header.stamp.to_nsec()) + '.png')
			cv2.imwrite(imagepath, cv2_img)


def write_cam_sensor_yaml(cam, output_yaml):
	with open(output_yaml, 'w') as f:
		f.write("# Sensor extrinsics wrt. the body-frame.\n")
		yaml.dump(cam, f, default_flow_style=None, sort_keys=False)


def write_imu_sensor_yaml(imu, output_yaml):
	with open(output_yaml, 'w') as f:
		f.write("# Sensor extrinsics wrt. the body-frame.\n")
		yaml.dump(imu, f, default_flow_style=None, sort_keys=False)
		f.write("""
# Units:
# accelerometer_noise_density: [ m / s^2 / sqrt(Hz) ] (accel "white noise")
# accelerometer_random_walk: [ m / s^3 / sqrt(Hz) ] (accel bias diffusion)
# gyroscope_noise_density: [ rad / s / sqrt(Hz) ] (gyro "white noise")
# gyroscope_random_walk: [ rad / s^2 / sqrt(Hz) ] (gyro bias diffusion)
""")


def get_imu_data(_):
	T_BS = np.identity(4).flatten().tolist()
	sensor_type = 'imu'
	rate_hz = 142

	data = {}
	data['sensor_type'] = sensor_type
	data['T_BS'] = {'cols': 4, 'rows': 4,'data': T_BS}
	data['rate_hz'] = rate_hz
	data['accelerometer_noise_density'] = 0.0006367367080238862
	data['accelerometer_random_walk'] = 0.0002643259445927608
	data['gyroscope_noise_density'] = 0.00028579943407611055
	data['gyroscope_random_walk'] = 7.544790215216806e-05

	return data


def get_cam_data(parsed_calib):
	T_cam_imu = np.array(parsed_calib['T_cam_imu'])
	T_imu_cam = np.linalg.inv(T_cam_imu)

	T_BS = T_imu_cam.flatten().tolist()
	sensor_type = 'camera'
	rate_hz = 15
	resolution = parsed_calib['resolution']
	camera_model = parsed_calib['camera_model']
	intrinsics = parsed_calib['intrinsics']
	distortion_model = "radial-tangential"
	distortion_coefficients = parsed_calib['distortion_coeffs']

	data = {}
	data['sensor_type'] = sensor_type
	data['T_BS'] = {'cols': 4, 'rows': 4,'data': T_BS}
	data['rate_hz'] = rate_hz
	data['resolution'] = resolution
	data['camera_model'] = camera_model
	data['intrinsics'] = intrinsics
	data['distortion_model'] = distortion_model
	data['distortion_coefficients'] = distortion_coefficients

	return data


if __name__ == "__main__":

	try:
		main()
	except KeyboardInterrupt:
		print('Interrupted.')
		try:
			sys.exit(0)
		except SystemExit:
			os._exit(0)