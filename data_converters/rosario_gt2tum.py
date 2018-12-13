#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import logging
import csv

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def main():
  """Main."""

  parser = argparse.ArgumentParser()
  parser.description = "Convert dataset ground-truth positions to TUM format"
  parser.add_argument(
    '-g',
    '--dataset-gt',
    required=True,
    help=("Ground-Truth positions (format: timestamp x y z)"))
  parser.add_argument(
      '-o',
      '--output',
      default="gt_tum.txt",
      required=False,
      help=("Output file"))

  # parse cmdline args
  parser_args = vars(parser.parse_args())
  in_dataset_gt_file = parser_args["dataset_gt"]
  out_new_format = parser_args["output"]

  # Ground-Truth file
  # Convert it to a
  if in_dataset_gt_file is None:
      logger.warn("Ground-Truth file not provided")
      return
  else:
      logger.info("moving the Ground-Truth information...")

      with open(in_dataset_gt_file) as inputFile:
        reader = csv.reader(inputFile, delimiter=' ')
        with open(out_new_format, 'w') as outputFile:
          writer = csv.writer(outputFile, delimiter=' ')
          for line in reader:

            timestamp = float(line[0])
            x_pos = line[1]
            y_pos = line[2]
            z_pos = line[3]

            # TUM format [x, y, z, w] identity quaternion
            quaternionOrientation = [0.0, 0.0, 0.0, 1.0]

            # create TUM pose with identity rotation matrix
            newRow = [timestamp, x_pos, y_pos, z_pos] + quaternionOrientation

            writer.writerow(newRow)

      # close files
      inputFile.close()
      outputFile.close()

if __name__=="__main__":
  main()
