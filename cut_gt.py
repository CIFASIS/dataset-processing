#!/usr/bin/env python3
import numpy as np
import argparse
import yaml
from rosbag.bag import Bag

def get_rosbag_info(path):
    return yaml.load(Bag(path, 'r')._get_yaml_info(), Loader=yaml.FullLoader)

def get_cut_index(ground_truth, end_date_in_seconds):
    return np.argmax(ground_truth[:,0] > end_date_in_seconds)

def cut_ground_truth(ground_truth, end_date_in_seconds):
    cut_index = get_cut_index(ground_truth, end_date_in_seconds)
    return ground_truth[:cut_index]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that cut a ground truth file to match the end date of a bagfile.')
    parser.add_argument('--gt', help='Ground truth file.')
    parser.add_argument('--input_bag', help='Rosbag file.')
    parser.add_argument('--output_gt', help='Output path (optional).')
    args = parser.parse_args()
    print(args)

    END_KEY = 'end'
    rosbag_info = get_rosbag_info(args.input_bag)
    end_date_in_seconds = rosbag_info[END_KEY]
    ground_truth = np.loadtxt(args.gt)

    if(args.output_gt):
        cut_gt = cut_ground_truth(ground_truth, end_date_in_seconds)
        np.savetxt(args.output_gt, cut_gt, fmt='%1.9f')
    else:
        print("Output will not be saved. You can use head -n <INDEX> <ORIGINAL_GT_FILE> to cut the file.")
        print("INDEX:")
        print(get_cut_index(ground_truth, end_date_in_seconds))
