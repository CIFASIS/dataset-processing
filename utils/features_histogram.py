#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import matplotlib.pyplot as plt
import argparse

if __name__ == "__main__":
  
    parser = argparse.ArgumentParser(description='Script that plot histogram of tracked features of a given sptam log file')
    parser.add_argument('--file', help='log file obtained from sptam')
    args = parser.parse_args()

    fr = open(args.file,"r")
    histogram = []
    for line in fr:
        data = line.split(" ")
        if(data[1] == "tk" and data[2] == "MeasurementCount:"):
            histogram.append(int(data[3][:-1]))


    plt.hist(histogram,bins = max(histogram))  # arguments are passed to np.histogram
    plt.title("Histogram of features tracked",fontsize=18)
    plt.xlabel("Number of times a feature was tracked",fontsize=16)
    plt.ylabel("Frequency",fontsize=16)
    plt.xticks(np.arange(0, max(histogram), 2.0))
    plt.show()