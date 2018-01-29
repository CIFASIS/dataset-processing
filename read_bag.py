#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rosbag
bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['fix']):
    print msg
bag.close()
