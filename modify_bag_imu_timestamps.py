#!/usr/bin/env python2
# -*- coding: utf-8 -*-




import rosbag
import rospy
import argparse
from os import path




def modify_rosbag_timestamps(input_bag, topic_to_modify, output_bag):
  '''
  Make rosbag timestamps temporally equidistant

  :param input_bag: Path to input bagfile
  :param topic_to_modify: name of topic
  :param output_bag: Path to output bagfile
  :return: None
  '''

  if path.exists(output_bag):
    raise ValueError("File already exists output_bag")

  with rosbag.Bag(output_bag, 'w') as outbag:
    bag = rosbag.Bag(input_bag)
    messages_iter = bag.read_messages(topics=[topic_to_modify])
    # Slow... A for loop could be used instead of list()
    messages = list(messages_iter)
    first = messages[0]
    last = messages[-1]
    samples = len(messages)
    # start_ts and end_ts used to be passed as int arguments in nanoseconds
    # These conversions should be improved, to_sec() should be used instead to_nsec()
    start_ts = first.timestamp.to_nsec()
    end_ts = last.timestamp.to_nsec()
    samples_m_1 = samples - 1
    print(start_ts, end_ts, float(samples_m_1)/float(end_ts-start_ts))
    assert samples_m_1 > 0
    to_sec = 1000000000
    bag_duration_sec = float(end_ts - start_ts) / to_sec
    dt = bag_duration_sec / samples_m_1
    ros_dt = rospy.Duration.from_sec(dt)
    rosbag_file = bag.read_messages()
    first = True
    for topic, msg, t in rosbag_file:
      if topic == topic_to_modify:
        assert msg.header.stamp == t
        if first:
          assert start_ts == t.to_nsec()
          new_t = t
          first = False
        else:
          new_t += ros_dt
        msg.header.stamp = new_t
        outbag.write(topic, msg, new_t)
      else:
        outbag.write(topic, msg, t)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("bag", help="Path to Rosbag file")
  parser.add_argument("--topic_to_modify", default="/imu", help="topic to modify")
  parser.add_argument("--output_bag", default="output.bag", help="Output bag")
  args = parser.parse_args()
  print(args)
  modify_rosbag_timestamps(args.bag, args.topic_to_modify, args.output_bag)
