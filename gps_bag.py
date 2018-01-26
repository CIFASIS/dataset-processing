#!/usr/bin/env python
import argparse
import rospy
import rosbag
from nmea_msgs.msg import Sentence #  sudo apt-get install ros-kinetic-nmea-msgs


if __name__ == "__main__":

  parser = argparse.ArgumentParser(description='Script that create a rosbag containing NMEA ROS sentences')
  parser.add_argument('gps_nmea', help='GPS log file')

  args = parser.parse_args()

  file = open(args.gps_nmea, 'r')

  bag = rosbag.Bag('test.bag', 'w')

  gps_topic = "nmea_sentence"
  seq = 0

  for line in file:  #for each line in raw file, take each part: time,id,msg

    # get timestamp
    timestamp = line.split(' ')[0]

    # get seconds
    seconds = int(timestamp.split('.')[0])

    # get milliseconds and convert them to nanoseconds
    nanoseconds = int(timestamp.split('.')[1]) * 1000 

    # create message
    data = Sentence()

    # message header
    data.header.seq = seq
    data.header.stamp.secs = seconds
    data.header.stamp.nsecs = nanoseconds
    data.header.frame_id = line.split(' ')[1][:-1]     #set frame_id = GPS-RTK

    # message data
    data.sentence = line.split(' ')[2][:-2]  # message from the type of NMEA sentence

    # remove checksum and CRLF
#    data.sentence = data.sentence[:-5]

    # add CR (Carriage Return) control character
#    data.sentence = data.sentence + '\r'
    # add LF (Line Feed) control character
#    data.sentence = data.sentence + '\n'


#    data.sentence = line [28:] # message from the type of NMEA sentence
#    print data.header.stamp.secs
#    print data.header.stamp.nsecs
#    print data.header.frame_id
#    print data.sentence
    bag.write(gps_topic, data, data.header.stamp)

    # increment seq number
    seq = seq + 1

  bag.close()
  file.close()
  quit()

