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
            \t -c clean temporal files generated during this execution \n\
            \t -h help" >&2
}

DROP=0
EQUIDISTANT_TS=0
ASL_FORMAT=0
CLEAN_TMPS=0

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
        c)  CLEAN_TMPS=1
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

  # generate rosbag
  echo "Creating $OUTPUT_DIR/$SEQ_NAME.bag"
  python $CURRENT_DIR/create_bagfile.py --images $SEQ_DIR/zed/ --imu $SEQ_DIR/imu.log --gps $SEQ_DIR/gps.log --calibration $DATASET_DIR/$CALIBRATION_NAME --odom $SEQ_DIR/odometry_raw.log --out $OUTPUT_DIR/$SEQ_NAME.bag

  # Drop last part of sequences 01 and 06.
  # This is done because we have found issues with the IMU that we have used to record data. (see README.md)
  if [ $DROP -eq 1 ] && ([ "$i" = "1" ] || [ "$i" = "6" ]) ; then
    echo "Dropping last part of $OUTPUT_DIR/$SEQ_NAME.bag"
    TMP_SEQ_NAME=drop_tmp_sequence0$i # IT SHOULD BE A UNIQUE NAME
    # Rename rosbag file (temporal bag)
    TMPBAG=$OUTPUT_DIR/$TMP_SEQ_NAME.bag
    mv $OUTPUT_DIR/$SEQ_NAME.bag $TMPBAG
    OUTBAG=$OUTPUT_DIR/$SEQ_NAME.bag
    START=$(rosbag info $TMPBAG -y -k start)
    DURATION=445
    # Run rosbag filter to cut the file
    rosbag filter $TMPBAG $OUTBAG "t.to_sec() <= $START + $DURATION"
    FINISHED_SUCCESSFULLY=$?
    if [ $FINISHED_SUCCESSFULLY -eq 0 ] && [ $CLEAN_TMPS -eq 1 ] ; then
      echo "Remove temporal file: $TMPBAG"
      rm $TMPBAG
    fi

  fi

  # Raw data have issues with timestamps. We fixed this issue by making them equidistant (see README.md).
  if [ $EQUIDISTANT_TS -eq 1 ] ; then
    echo "Creating rosbag with equidistant timestamps $OUTPUT_DIR/$SEQ_NAME.bag"
    TMP_SEQ_NAME=eq_tmp_sequence0$i  # IT SHOULD BE A UNIQUE NAME
    # Rename rosbag file (temporal bag)
    TMPBAG=$OUTPUT_DIR/$TMP_SEQ_NAME.bag
    mv $OUTPUT_DIR/$SEQ_NAME.bag $TMPBAG
    OUTBAG=$OUTPUT_DIR/$SEQ_NAME.bag
    # Create a new rosbag with equidistant timestamps
    python $CURRENT_DIR/modify_bag_imu_timestamps.py $TMPBAG --output_bag $OUTBAG
    FINISHED_SUCCESSFULLY=$?
    if [ $FINISHED_SUCCESSFULLY -eq 0 ] && [ $CLEAN_TMPS -eq 1 ] ; then
      echo "Remove temporal file: $TMPBAG"
      rm $TMPBAG
    fi
  fi

  # Generate ASL format from rosbag.
  if [ $ASL_FORMAT -eq 1 ] ; then
    echo "Save $OUTPUT_DIR/$SEQ_NAME.bag as ASL format"
    INBAG=$OUTPUT_DIR/$SEQ_NAME.bag
    ASL_SEQ_DIR=$OUTPUT_DIR/$ASL_SEQ_NAME
    python $CURRENT_DIR/rosario2euroc.py $INBAG --calib $DATASET_DIR/$CALIBRATION_NAME --output $ASL_SEQ_DIR
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
