#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import logging
import csv
import numpy as np
from plyfile import PlyData, PlyElement

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def main():
  """Main."""

  parser = argparse.ArgumentParser()
  parser.description = "Convert TUM trajectory to PLY format"
  parser.add_argument(
  '-t',
  '--trajectory',
  required=True,
  help=("Trajectory (TUM format: timestamp x y z)"))
  parser.add_argument(
      '-o',
      '--output',
      default="output.ply",
      required=False,
      help=("Output PLY file"))

  # parse cmdline args
  parser_args = vars(parser.parse_args())
  in_dataset_gt_file = parser_args["trajectory"]
  out_new_format = parser_args["output"]

  # Ground-Truth file
  # Convert it to a
  if in_dataset_gt_file is None:
      logger.warn("Trajectory file not provided")
      return
  else:
      logger.info("Converting Trajectory from TUM file {} to PLY file {}".format(in_dataset_gt_file, out_new_format))

      with open(in_dataset_gt_file) as inputFile:
        reader = csv.reader(inputFile, delimiter=' ')
        # with open(out_new_format, 'w') as outputFile:
          # writer = csv.writer(outputFile, delimiter='')

          # writer.writerow("ply")
          # writer.writerow('format ascii 1.0')
          # data = list(reader)
          # row_count = len(data)
          # writer.writerow('element vertex ' + str(row_count))
        vertices_list = []
        for line in reader:

          timestamp = float(line[0])
          x_pos = line[1]
          y_pos = line[2]
          z_pos = line[3]

          # TUM format [x, y, z, w] identity quaternion
          x_ori_q = line[4]
          y_ori_q = line[5]
          z_ori_q = line[6]
          w_ori_q = line[7]

          # store poses as tuples (ply vertices)
          vertices_list.append((x_pos, y_pos, z_pos))

      # convert to PLY
      vertices = np.empty(len(vertices_list), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
      vertices[:] = vertices_list
      el = PlyElement.describe(vertices, 'vertex')
      PlyData([el], text=True).write(out_new_format)

      # close file
      inputFile.close()

if __name__=="__main__":
  main()
