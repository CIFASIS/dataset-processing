#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import numpy as np
sptam = []
sptam_x = []
sptam_y = []
sptam_z = []

def callback_sptam(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #y = math.sqrt(data.pose.pose.position.z**2 + data.pose.pose.position.y**2)
    if (data.header.seq > 3): 
        x=data.pose.pose.position.x
        y=data.pose.pose.position.y
        z=data.pose.pose.position.z
        sptam_pos = np.array([x,y,z])
        sptam_pos_norm = sptam_pos/np.linalg.norm(sptam_pos)
        sptam.append(sptam_pos_norm)
        sptam_x.append(sptam_pos_norm[0])
        sptam_y.append(sptam_pos_norm[1])
        sptam_z.append(sptam_pos_norm[2])

    #plt.scatter(data.pose.pose.position.x,data.pose.pose.position.z)
    #plt.pause(0.05)

    #plt.show()

rospy.init_node('sptam_listener', anonymous=True)
rospy.Subscriber("/sptam/robot/pose", PoseWithCovarianceStamped, callback_sptam)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
sptam_mean = np.array([np.mean(sptam_x),np.mean(sptam_y),np.mean(sptam_z)])
print sptam_mean
fs = open("/home/martin/Desktop" +'/' + "sptam.log",'w+')
fs.write("\r\n")
fs.write(str(sptam_mean))
fs.write("\r\n")
fs.write("\r\n") 
fs.write("\r\n")     
for value in sptam:
    fs.write(str(value) + '\r\n')