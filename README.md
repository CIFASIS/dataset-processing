gps_bag needs building in catkin_ws nmea_msgs 

use MarkDown language!!!!

Use the S-PTAM branch https://gitlab.com/taihu/sptam/tree/ras_2017_sptam_full_working to make S-PTAM work properly on the


rus S-PTAM on Rosario dataset
roscore &
rosparam set use_sim_time true
roslaunch sptam zed_bag.launch
rosbag play --clock <sequenceXX.bag>

[crear configuración para levantar con rviz]

Plotters
el Plot de S-PTAM plot-paths anda para mostrar la trayectoria bien de S-PTAM. Falta el script de comparación con el GT y comparación con otros métodos
Podemos probar con ORB-SLAM.

Transformar datos de GT a formato EuRoC y a KITTI para poder comparar fácilmente (hacer script)

Utilizar software de ploteo EVO: https://github.com/MichaelGrupp/evo
Instalarlo para python 2.7 para esto hacer:
sudo pip2.7 install evo --upgrade --no-binary evo


# RUN ORB-SLAM2 in Rosario dataset

rosbag play --clock sequence01.bag /stereo/left/image_raw:=/camera/left/image_raw /stereo/right/image_raw:=/camera/right/image_raw

rosrun ORB_SLAM2 Stereo /home/taihu/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/taihu/src/dataset-processing/slam_configs/orbslam.yaml true

la trayectoria correcta esta en: FrameTrajectory_TUM_Format.txt











