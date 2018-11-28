import argparse
import logging
import csv

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def main():
  """Main."""

  parser = argparse.ArgumentParser()
  parser.description = "Convert S-PTAM log file to KITTI format"
  parser.add_argument(
    '-s',
    '--sptam-log',
    required=True,
    help=("S-PTAM log file (format: TRACKED_FRAME_POSE: timestamp frame_number r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz Cov00 .. Covxx)"))
  parser.add_argument(
      '-o',
      '--output',
      default="sptam_kitti.txt",
      required=False,
      help=("Output file"))

  # parse cmdline args
  parser_args = vars(parser.parse_args())
  in_sptam_log_file = parser_args["sptam_log"]
  out_kitti_format = parser_args["output"]

  # Ground-Truth file
  # Convert it to a
  if in_sptam_log_file is None:
      logger.warn("S-PTAM log file not provided")
      return
  else:
      logger.info("Converting S-PTAM log file to KITTI format...")

      with open(in_sptam_log_file) as inputFile:
        with open(out_kitti_format, 'w') as outputFile:
          writer = csv.writer(outputFile, delimiter=' ')
          reader = csv.reader(inputFile, delimiter=' ')
          for line in reader:
            timestamp = line[0]
            if "TRACKED_FRAME_POSE" == line[0]:
              timestamp = line[1]
              pose = line[2:14]

              # create kitti pose with identity rotation matrix
              writer.writerow(pose)

      # close files
      inputFile.close()
      outputFile.close()

if __name__=="__main__":
  main()
