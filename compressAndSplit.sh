#!/bin/bash

for i in `seq 1 6`; do
    SEQ_NAME=sequence0$i
    SEQ_DIR=$SEQ_NAME/
    echo "processing sequence 0"$i
    # compress the sequence directory 
    tar cvzf $SEQ_NAME.tar.gz $SEQ_DIR

    # split the compressed sequence dile
    split --bytes=2GB -d $SEQ_NAME.tar.gz $SEQ_NAME.tar.gz.

    # compress rosbag sequence
    rosbag compress $SEQ_NAME.bag

    # split the compressed rosbag 
    split --bytes=2GB -d $SEQ_NAME.bag $SEQ_NAME.bag.

    # generate checksum
    md5sum $SEQ_NAME.tar.gz >> checksum.txt
    md5sum $SEQ_NAME.bag >> checksum.txt
done
