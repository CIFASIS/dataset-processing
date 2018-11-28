import argparse
import logging
import csv

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def main():
  """Main."""

  parser = argparse.ArgumentParser()
  parser.description = "Convert dataset ground-truth positions to KITTI format"
  parser.add_argument(
    '-g',
    '--dataset-gt',
    required=True,
    help=("Ground-Truth positions (format: timestamp x y z)"))
  parser.add_argument(
      '-o',
      '--output',
      default="output.txt",
      required=False,
      help=("Path to the top-level input directory for conversion"))

  # parse cmdline args
  parser_args = vars(parser.parse_args())
  in_dataset_gt_file = parser_args["dataset_gt"]
  out_gt_kitti_format = parser_args["output"]

  # Ground-Truth file
  # Convert it to a
  if in_dataset_gt_file is None:
      logger.warn("Ground-Truth file not provided")
      return
  else:
      logger.info("moving the Ground-Truth information...")

      with open(in_dataset_gt_file) as inputFile:
        with open(out_gt_kitti_format, 'w') as outputFile:
          writer = csv.writer(outputFile, delimiter=' ')
          reader = csv.reader(inputFile, delimiter=' ')
          for line in reader:
            timestamp = line[0]
            x_pos = line[1]
            y_pos = line[2]
            z_pos = line[3]

            # create kitti pose with identity rotation matrix
            newRow = [1.0, 0.0 , 0.0, x_pos, 0.0, 1.0, 0.0, y_pos, 0.0, 0.0, 1.0, z_pos]
            writer.writerow(newRow)

      # close files
      inputFile.close()
      outputFile.close()

if __name__=="__main__":
  main()
