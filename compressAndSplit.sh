#!/bin/bash

OUTPUT_DIR=$HOME/datasets/robot_desmalezador
DATASET_DIR=$HOME/datasets/robot_desmalezador

for i in `seq 4 4`; do
    SEQ_NAME=sequence0$i
    SEQ_DIR=$DATASET_DIR/$SEQ_NAME
    echo "processing $SEQ_NAME in directory " $DATASET_DIR

    # generate rosbag
    python create_bagfile.py --images $SEQ_DIR/zed/ --imu $SEQ_DIR/imu.log --gps $SEQ_DIR/gps.log --calibration $SEQ_DIR/calibration.yaml --odom $SEQ_DIR/odometry_raw.log --out $OUTPUT_DIR/$SEQ_NAME.bag

    # compress the sequence directory 
    tar cvzf $SEQ_NAME.tar.gz $SEQ_DIR

    # compress rosbag sequence
    rosbag compress $SEQ_NAME.bag

    if [ "$i" = "1" ] || [ "$i" = "2" ] || [ "$i" = "5" ] || [ "$i" = "6" ]; then

      # split the compressed sequence file
      split --bytes=2GB -d $SEQ_NAME.tar.gz $SEQ_NAME.tar.gz.

      # split the compressed rosbag
      split --bytes=2GB -d $SEQ_NAME.bag $SEQ_NAME.bag.
    fi

    # generate checksum
    md5sum $SEQ_NAME.tar.gz >> checksum.txt
    md5sum $SEQ_NAME.bag >> checksum.txt
done
