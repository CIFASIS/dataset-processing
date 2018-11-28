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

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def main():
  """Main."""

  parser = argparse.ArgumentParser()
  parser.description = "Convert S-PTAM log file to TUM format"
  parser.add_argument(
    '-s',
    '--sptam-log',
    required=True,
    help=("S-PTAM log file (format: TRACKED_FRAME_POSE: timestamp frame_number r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz Cov00 .. Covxx)"))
  parser.add_argument(
      '-o',
      '--output',
      default="sptam_tum.txt",
      required=False,
      help=("Output file"))

  # parse cmdline args
  parser_args = vars(parser.parse_args())
  in_sptam_log_file = parser_args["sptam_log"]
  out_new_format = parser_args["output"]

  # Convert S-PTAM log  file to TUM format
  if in_sptam_log_file is None:
      logger.warn("S-PTAM log file not provided")
      return
  else:
      logger.info("Converting S-PTAM log file to TUM format...")

      with open(in_sptam_log_file) as inputFile:
        with open(out_new_format, 'w') as outputFile:
          writer = csv.writer(outputFile, delimiter=' ')
          reader = csv.reader(inputFile, delimiter=' ')
          timestamp = 0
          for line in reader:

            if "FRAME_TIMESTAMP" == line[0]:
              sec = np.uint64(line[2])
              nsec = np.uint64(line[3])
              time = Time(sec,nsec)
              timestamp = time.to_sec()

            if "TRACKED_FRAME_POSE" == line[0]:

              # get sptam pose
              sptam_pose = np.array(line[2:14])
              transformation = np.reshape( sptam_pose.astype(float), (3, 4))
              traslation = transformation[:,3]
              rotation = transformation[:3,:3]

              # quaternion has the form [w, x, y, z]
              quaternion = tf.quaternion_from_matrix(rotation)
              # TUM quaternion format is [x, y, z, w]
              quaternion = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])

              # create TUM pose with identity rotation matrix
              newRow = [timestamp] + list(traslation) + list(quaternion)
              writer.writerow(newRow)

              # create TUM pose with identity rotation matrix
              writer.writerow(newRow)

      # close files
      inputFile.close()
      outputFile.close()

if __name__=="__main__":
  main()
