#!/bin/bash

OUTPUT_DIR=$HOME/datasets/robot_desmalezador
DATASET_DIR=$HOME/datasets/robot_desmalezador

# move to dataset directory (this is important for the compression path)
cd $DATASET_DIR

for i in `seq 1 6`; do
  SEQ_NAME=sequence0$i
  SEQ_DIR=$DATASET_DIR/$SEQ_NAME
  echo "Processing $SEQ_NAME in directory $DATASET_DIR/"

  # generate rosbag
  echo "Creating $OUTPUT_DIR/$SEQ_NAME.bag"
  python create_bagfile.py --images $SEQ_DIR/zed/ --imu $SEQ_DIR/imu.log --gps $SEQ_DIR/gps.log --calibration $SEQ_DIR/calibration.yaml --odom $SEQ_DIR/odometry_raw.log --out $OUTPUT_DIR/$SEQ_NAME.bag

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
done
echo "Everything done"
