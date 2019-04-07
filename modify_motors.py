#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description='Script that takes motors messages and create a cleaner with only basic info log file')
  parser.add_argument('--input', help='old motors log file')
  parser.add_argument('--output', help='new motors log file')
  args = parser.parse_args()
  fr = open(args.input,"r") 
  fw = open(args.output,"w+")
  for line in fr:
    sentence = line.split(" ")
    data = sentence[2].split(",")
    vel_1 = float(data[9])
    vel_2 = float(data[13])
    angle = float(data[16])+5.67 # offset estimated in the direction angle
    direction = data[17][:-3]
    if direction == "0": # to change direction ford = 0 back = 1 to ford = 1 back = -1
      direction = "1"
    else:
      direction = "-1"
    fw.write(sentence[0] + " " + str(vel_1) + " " + str(vel_2) + " " + str(angle) + " " + str(direction) + "\r\n")
  fr.close()
  fw.close()  
     
