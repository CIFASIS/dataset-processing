#!/bin/bash

OUTPUT_DIR=$HOME/datasets/robot_desmalezador
DATASET_DIR=$HOME/datasets/robot_desmalezador

# get full directory name of the script no matter where it is being called from.
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

function echoUsage()
{
    echo -e "Usage: ./compressAndSplit.sh [FLAG] \n\
            \t -e with equidistant timestamps \n\
            \t -d drop last part of seq. 01 and 06 (see README.md) \n\
            \t -a save files as ASL format \n\
            \t -h help" >&2
}

DROP=0
EQUIDISTANT_TS=0
ASL_FORMAT=0

while getopts "hdeac" opt; do
    case "$opt" in
        h)
            echoUsage
            exit 0
            ;;
        d)  DROP=1
            ;;
        e)  EQUIDISTANT_TS=1
            ;;
        a)  ASL_FORMAT=1
            ;;
        *)
            echoUsage
            exit 1
            ;;
    esac
done

# move to dataset directory (this is important for the compression path)
cd $DATASET_DIR

for i in `seq 1 6`; do
  SEQ_NAME=sequence0$i
  ASL_SEQ_NAME=asl_$SEQ_NAME
  CALIBRATION_NAME=calibration0$i.yaml
  SEQ_DIR=$DATASET_DIR/$SEQ_NAME
  echo "Processing $SEQ_NAME in directory $DATASET_DIR/"

  PY_ARGS="--images $SEQ_DIR/zed/ --imu $SEQ_DIR/imu.log --gps $SEQ_DIR/gps.log --calibration $DATASET_DIR/$CALIBRATION_NAME --odom $SEQ_DIR/odometry_raw.log --out $OUTPUT_DIR/$SEQ_NAME.bag"
  # Drop last part of sequences 01 and 06.
  # This is done because we have found issues with the IMU that we have used to record data. (see README.md)
  if [ $DROP -eq 1 ] ; then
    if [ "$i" = "1" ] ; then
      PY_ARGS="${PY_ARGS} --max_duration 445.103"
    fi
    if [ "$i" = "6" ] ; then
      PY_ARGS="${PY_ARGS} --max_duration 444.993"
    fi
  fi
  # Raw data have issues with timestamps. We fixed this issue by making them equidistant (see README.md).
  if [ $EQUIDISTANT_TS -eq 1 ] ; then
    PY_ARGS="${PY_ARGS} --eqdistant_imu"
  fi
  # Generate ASL format from rosbag.
  if [ $ASL_FORMAT -eq 1 ] ; then
    ASL_SEQ_DIR=$OUTPUT_DIR/$ASL_SEQ_NAME
    PY_ARGS="${PY_ARGS} --save_asl_format $ASL_SEQ_DIR"
  fi

  # generate rosbag
  echo "Creating $OUTPUT_DIR/$SEQ_NAME.bag"
  python $CURRENT_DIR/create_bagfile.py $PY_ARGS

  if [ $ASL_FORMAT -eq 1 ] ; then
    echo "Compressing $ASL_SEQ_DIR"
    tar cf - $ASL_SEQ_NAME -P | pv -s $(du -sb $ASL_SEQ_DIR | awk '{print $1}') | gzip > $OUTPUT_DIR/$ASL_SEQ_NAME.tar.gz
  fi

  # compress the sequence directory
  echo "Compressing $SEQ_DIR"
  tar cf - $SEQ_NAME -P | pv -s $(du -sb $SEQ_DIR | awk '{print $1}') | gzip > $OUTPUT_DIR/$SEQ_NAME.tar.gz

  # compress rosbag sequence
  echo "Compressing $OUTPUT_DIR/$SEQ_NAME.bag"
  rosbag compress $OUTPUT_DIR/$SEQ_NAME.bag

  if [ "$i" = "1" ] || [ "$i" = "2" ] || [ "$i" = "5" ] || [ "$i" = "6" ]; then

    # split the compressed sequence file
    echo "Splitting $OUTPUT_DIR/$SEQ_NAME.tar.gz"
    split --bytes=2GB -d $OUTPUT_DIR/$SEQ_NAME.tar.gz $OUTPUT_DIR/$SEQ_NAME.tar.gz.

    # split the compressed rosbag
    echo "Splitting $OUTPUT_DIR/$SEQ_NAME.bag"
    split --bytes=2GB -d $OUTPUT_DIR/$SEQ_NAME.bag $OUTPUT_DIR/$SEQ_NAME.bag.
  fi

  # generate checksum
  echo "Computing checksums"
  md5sum $OUTPUT_DIR/$SEQ_NAME.tar.gz >> $OUTPUT_DIR/checksum.txt
  md5sum $OUTPUT_DIR/$SEQ_NAME.bag >> $OUTPUT_DIR/checksum.txt
  if [ $ASL_FORMAT -eq 1 ] ; then
    md5sum $OUTPUT_DIR/$ASL_SEQ_NAME.tar.gz >> $OUTPUT_DIR/checksum.txt
  fi

done
echo "Everything done"
