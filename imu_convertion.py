#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import argparse

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Script that creates an imu bin file that can be used in dataset2bag')
	parser.add_argument('imu_raw', help='imu raw log')
	parser.add_argument('imu_new', help='imu new log')

	args = parser.parse_args()
	orientation = "1 0 0 0 1 0 0 0 1"
	fr = open(args.imu_raw,"r") #information obtained from sensor
	fw = open(args.imu_new,"w") #new imu file to use with dataset2bag

#since number of characters of the values may change lines are splitted usings spaces as delimiters	
	for line in fr:
		splitted_line = line.split(" ")
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

		fw.write(seconds + " " + nanoseconds + " " + str(Gx_rads) + " " + str(Gy_rads) + " " + str(Gz_rads) + " " + str(Tx_meters) + " " + str(Ty_meters) + " " + str(Tz_meters) + " " + orientation + "\n")

	fr.close()
	fw.close()


