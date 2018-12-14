#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import logging
import csv
import numpy as np
import sys
sys.path.insert(0, '../')
from utils import transformations as tf
from utils.Time import Time
import rosbag

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def main():
  """Main."""

  parser = argparse.ArgumentParser()
  parser.description = "Convert a KITTI ground-truth file to TUM format. The timestamps are extracted from the kitti rosbag used to run the sequence"
  parser.add_argument(
    '-k',
    '--kitti-gt',
    required=True,
    help=("KITTI ground-truth file (format: r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz)"))
  parser.add_argument(
    '-r',
    '--rosbag-file',
    required=True,
    help=("Rosbag file to extract the timestamp of frames."))
  parser.add_argument(
      '-o',
      '--output',
      default="gt_tum_kittiXX.txt",
      required=False,
      help=("Output file"))

  # parse cmdline args
  parser_args = vars(parser.parse_args())
  in_kitti_gt_file = parser_args["kitti_gt"]
  in_rosbag_file = parser_args["rosbag_file"]
  out_new_format = parser_args["output"]

  # Convert KITTI ground-truth with times file to TUM format
  if in_kitti_gt_file is None:
      logger.warn("KITTI ground-truth file not provided")
      return
  else:
      logger.info("Converting KITTI ground-truth file to TUM format...")

      # get timestamps from rosbag
      timestamps = []
      for topic, msg, t in rosbag.Bag(in_rosbag_file, 'r').read_messages():

        # set seq for left camera_info message
        if topic == "kitti_stereo/left/image_rect":
          timestamp = msg.header.stamp.to_sec()
          timestamps.append(timestamp)

      with open(in_kitti_gt_file) as kitti_gt_file:
          with open(out_new_format, 'w') as outputFile:
            writer = csv.writer(outputFile, delimiter=' ')
            kitti_gt_reader = csv.reader(kitti_gt_file, delimiter=' ')

            # each line in the kitti is a pose
            for kitti_gt_line, timestamp in zip(kitti_gt_reader, timestamps):

                # get pose
                kitti_pose = np.array(kitti_gt_line)
                transformation = np.reshape( kitti_pose.astype(float), (3, 4))
                traslation = transformation[:,3]
                rotation = transformation[:3,:3]

                # quaternion has the form [w, x, y, z]
                quaternion = tf.quaternion_from_matrix(rotation)
                # TUM quaternion format is [x, y, z, w]
                quaternion = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])

                # create TUM format
                newRow = [timestamp] + list(traslation) + list(quaternion)
                writer.writerow(newRow)

      # check that ground-truth and rosbag timestamps have the same length
      assert kitti_gt_reader.line_num == len(timestamps)

      # close files
      kitti_gt_file.close()
      outputFile.close()

if __name__=="__main__":
  main()
