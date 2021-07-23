*dataset-processing* is a set of scripts to process, convert and visualize the [Rosario dataset](http://www.cifasis-conicet.gov.ar/robot)


# Disclaimer
This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

[//]: # (Use the S-PTAM branch https://gitlab.com/taihu/sptam/tree/ras_2017_sptam_full_working to make S-PTAM work properly on the Rosario dataset.)


# Ground-truth
The Ground-Truth is generated directly with the GPS-RTK positional measurements. To this end firstly the NMEA are converted to the format **timestamp x, y, z** using

	python <dataset-processing>/nmea2gps.py --input <gps_log> -output <rosatio_gt>

Then, this format could be converted to any desired standard format as TUM or KITTI using the respectively converter:

	python rosario_gt2tum.py -g <rosatio_gt> -o <rosario_gt_kitti>
	python rosario_gt2kitti.py -g <rosatio_gt> -o <rosario_gt_tum>


[//]: # (The S-PTAM log file has the estimated pose indicated with the keyword TRACKED_FRAME_POSE, so:)
[//]: # (TRACKED_FRAME_POSE: timestamp frame_number r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz Cov00 .. Covxx)

# Visualization tools

## GPS visualization with mapviz

For the visualization of GPS measurements on ROS we recomend [mapviz](https://github.com/swri-robotics/mapviz)

	roscore &
	roslaunch <dataset-processing>/visualize_navsatfix.launch
	rosbag play <rosbag>

# Tested SLAM systems

## robot_localization
	roscore &
	rosparam set use_sim_time true
	roslaunch <dataset-processing>/robot_localization/dual_ekf_navsat.launch
	rosbag play --clock <sequenceXX.bag>


## S-PTAM

	roscore &
	rosparam set use_sim_time true
	roslaunch <dataset-processing>/slam_configs/sptam_zed_bag.launch
	rosbag play --clock <sequenceXX.bag>

## ORB-SLAM2

	roscore &
	rosparam set use_sim_time true
	rosbag play --clock sequence01.bag /stereo/left/image_raw:=/camera/left/image_raw /stereo/right/image_raw:=/camera/right/image_raw
	rosrun ORB_SLAM2 Stereo ~/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt <dataset-processing>/slam_configs/orbslam_ros.yaml true
	rosbag play --clock <sequenceXX.bag>

[//]: # (The output trajectory is stored as FrameTrajectory_TUM_Format.txt)

[//]: # (If ORB-SLAM crashes, it is possible to recover the estimated trajectory from the file tracked_poses_tum.log)

# Dataset evaluation

We recomend the [evo](https://github.com/MichaelGrupp/evo) evaluation tool.

Use the python 2.7 version:

	sudo pip2.7 install evo --upgrade --no-binary evo


Examples:

evo_traj tum --ref=gt_tum_02.txt FrameTrajectory_TUM_Format.txt -p --plot_mode=xyz --align

evo_traj tum --ref=gt_tum_02.txt FrameTrajectory_TUM_Format.txt -p --plot_mode=xyz --align

[//]: # (# Script to upload data)

[//]: # (compressAndSplit.sh)

# Convert raw data to bagfiles and ASL format
## Install dependencies

	sudo apt install pv

## Run convertion script
A bash script is provided to generate rosbag files from raw data.
In addition, a tar containing files as ASL format is also generated for each sequence.
To do this, just run:
```
./compressAndSplit.sh -deac
```
Flags (see below for more details):

* `-d`: drop last part of sequences 01 and 06.
* `-e`: fix timestamp issue by making them equidistant.
* `-a`: generate ASL format.

Raw files and generated files can be found in the [official Rosario dataset site](http://www.cifasis-conicet.gov.ar/robot).

We have experienced issues when collecting data with the IMU we have used to record the dataset.
After approximately 458 seconds of recording, erroneous measurements are obtained for a few seconds.
We performed several experiments to check this hardware issue.

Duration of sequences 01 and 06 is greater than 458 seconds. We provide a `-d` flag to cut sequences 01 and 06,
discarding the last part of these sequences. Duration of the generated rosbags is approximately 445 seconds
(this value can be easily edited in `compressAndSplit.sh`).

Additionally, an issue related to the timestamps was fixed. We found that multiple rows were labeled with the same timestamp.
To solve this, `-e` flag allows to fix timestamps making them equidistant.
See `modify_bag_imu_timestamps.py` for more details.