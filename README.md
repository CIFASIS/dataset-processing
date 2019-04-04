dataset-processing is a set of scripts to process, convert and visualize the [Rosario dataset](http://www.cifasis-conicet.gov.ar/robot)


# Disclaimer
This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

gps_bag needs building in catkin_ws nmea_msgs 


Use the S-PTAM branch https://gitlab.com/taihu/sptam/tree/ras_2017_sptam_full_working to make S-PTAM work properly on the Rosario dataset.



# Ground-truth
The Ground-Truth is generated directly with the GPS-RTK positional measurements. To this end firstly the NMEA are converted to the format **timestamp x, y, z** using

	python nmea2gps.py --input <gps_log> -output <rosatio_gt>

Then, this format could be converted to any desired standard format as TUM or KITTI using the respectively converter:

	python rosario_gt2tum.py -g <rosatio_gt> -o <rosario_gt_kitti>
	python rosario_gt2kitti.py -g <rosatio_gt> -o <rosario_gt_tum>


[//]: # (The S-PTAM log file has the estimated pose indicated with the keyword TRACKED_FRAME_POSE, so:)
[//]: # (TRACKED_FRAME_POSE: timestamp frame_number r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz Cov00 .. Covxx)

# Visualization tools

## GPS visualization with mapviz

For the visualization of GPS measurements on ROS we recomend [mapviz](https://github.com/swri-robotics/mapviz)


	roscore &
	rosbag play <rosbag>
	roslaunch visualize_navsatfix.launch


# RUN S-PTAM in Rosario dataset

	roscore &
	rosparam set use_sim_time true
	roslaunch sptam zed_bag.launch
	rosbag play --clock <sequenceXX.bag>


# RUN ORB-SLAM2 in Rosario dataset

	roscore &
	rosparam set use_sim_time true
	rosbag play --clock sequence01.bag /stereo/left/image_raw:=/camera/left/image_raw /stereo/right/image_raw:=/camera/right/image_raw
	rosrun ORB_SLAM2 Stereo /home/taihu/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt dataset-processing/slam_configs/orbslam.yaml true

The output trajectory is stored as FrameTrajectory_TUM_Format.txt

If ORB-SLAM crashes, it is possible to recover the estimated trajectory from the file tracked_poses_tum.log

# Dataset evaluation

We recoend the [evo](https://github.com/MichaelGrupp/evo) evaluation tool.

Use the python 2.7 version:

	sudo pip2.7 install evo --upgrade --no-binary evo


Exaples:

evo_traj tum --ref=gt_tum_02.txt FrameTrajectory_TUM_Format.txt -p --plot_mode=xyz --align

evo_traj tum --ref=gt_tum_02.txt FrameTrajectory_TUM_Format.txt -p --plot_mode=xyz --align

# Additional content

## Script to upload data

compressAndSplit.sh


# CHANGELOG

[Create rviz configuration to show data]
