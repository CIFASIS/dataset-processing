import argparse
import logging
import csv

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def main():
  """Main."""

  parser = argparse.ArgumentParser()
  parser.description = "Convert dataset ground-truth positions to EuRoC format"
  parser.add_argument(
    '-g',
    '--dataset-gt',
    required=True,
    help=("Ground-Truth positions (format: timestamp x y z)"))
  parser.add_argument(
      '-o',
      '--output',
      default="gt_euroc.txt",
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
          writer = csv.writer(outputFile, delimiter=',')
          for line in reader:
            # convert seconds to nanoseconds
            timestamp = float(line[0]) * 1000000000 # we are losing precision here
            x_pos = line[1]
            y_pos = line[2]
            z_pos = line[3]

            # identity quaternion
            quaternionOrientation = [1, 0.0, 0.0, 0.0]
            linearVelocity = [0.0, 0.0, 0.0]
            biasGyros = [0.0, 0.0, 0.0]
            biasAccelerometer = [0.0, 0.0, 0.0]

            # create euroc pose with identity rotation matrix
            newRow = [timestamp, x_pos, y_pos, z_pos] + quaternionOrientation + linearVelocity + biasGyros + biasAccelerometer
            writer.writerow(newRow)

      # close files
      inputFile.close()
      outputFile.close()

if __name__=="__main__":
  main()
