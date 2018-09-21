#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rosbag
import argparse
from nav_msgs.msg import Path
if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description='Script that takes x,y and timestamps from a file and put it in ROSBAG')
  parser.add_argument('gps', help='gps log file')
  parser.add_argument('outbag', help='output bag file')
  args = parser.parse_args()

  inFile = open(args.gps, 'r')
  bag = rosbag.Bag(args.outbag, 'w')
  seq = 0
  for line in inFile:
  	
  	path = Path()
  	sentence = line.split(' ')
  	seconds, nanoseconds = sentence[0].split('.')
  	nanoseconds = nanoseconds + "000"
  	path.header.stamp.secs = seconds
  	path.header.stamp.nsecs = nanoseconds
  	path.header.seq = seq
  	path.pose




