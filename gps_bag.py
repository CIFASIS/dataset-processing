#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rosbag
from std_msgs.msg import Int32, String #not needed, only for testing with basic example
from nmea_msgs.msg import Sentence

data = Sentence()
bag = rosbag.Bag('test.bag', 'w')
F = open("/home/martin/datasets/2017-12-26_11:03:06/gps/2017-12-26_11:03:06.log",'r')


for line in F:  #for each line in raw file, take each part: time,id,msg
	data.header.stamp.secs = line[0:10] #spliting seconds
	data.header.stamp.nsecs = line [11:17] #spliting nanoseconds, is this nanoseconds? or it needs a convertion?
	data.header.frame_id = line [18:25] #frame_id = GPS-RTK
	data.sentence = line [28:] # message from the tipe of NMEA sentence
	print data.header.stamp.secs
	print data.header.stamp.nsecs
	print data.header.frame_id
	print data.sentence
	bag.write(data.header.frame_id,data.sentence, data.header.stamp)  # Here is the problem, data.sentence doesnt have type
	
bag.close()
F.close()
quit()

'''
a=String()
a.data = "erfdf"
print "d3"
bag.write(line.header.frame_id,a, line.header.stamp)
print "d4"
#bag.write('numbers', i)
bag.close()
'''



