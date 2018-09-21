#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from PIL import Image
import argparse
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that takes images imu and gps along with the calibration info to create a rosbag')
    parser.add_argument('--images', help='imu log file')
    parser.add_argument('--out', help='imu log file')
    args = parser.parse_args()
    for filename in sorted(os.listdir(args.images)): 
        image = Image.open(args.images + str(filename))
        newImage = image.resize((336, 188))
        newImage.save(args.out + filename)
    print "finished"
