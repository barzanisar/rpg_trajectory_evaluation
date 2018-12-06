#!/usr/bin/python
"""
Created on Mon May 30 19:11:47 2016

@author: zichao
"""

import rosbag
import rospy
import cv2
import os, glob
import yaml
import argparse
from decimal import *

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo


def synthetic_to_bags(output_dir,
                      img_topic='/camera/image_raw', camera_name = 'Camera_L',
                     file_extension = '.png'): #info_topic='/camera/camera_info' output='out.bag',
    #image_list_name = os.path.join(dataset_dir, 'images.txt')
    #calib_file_name = os.path.join(dataset_dir, '../calib.yaml')
    image_dir = output_dir + '/Camera_L_NYC_Subway_Station/*'
    print (image_dir)
    output = camera_name + '_40Hz.bag'
    out_bag_name = os.path.join(output_dir, output)
    print(out_bag_name)
    bridge = CvBridge()

    #image_list = open(image_list_name, 'r')
    out_bag = rosbag.Bag(out_bag_name, 'w')

    #calib_yaml = yaml.load(open(calib_file_name, 'r'))
    #camera = calib_yaml['cameras'][0]['camera']
    #intrinsics = camera['intrinsics']['data']

    #print camera
        #timestamp_sec =  Decimal(float(''.join(c for c in img_name if c.isdigit()))) / Decimal (1e6)
        
        #print (timestamp_sec)
        #print("{:.6f}".format(timestamp_sec))
        #print (Decimal(timestamp_sec).quantize(Decimal('.00001'), rounding=ROUND_DOWN))
        #print (Decimal(round(timestamp_sec,6)))
        #print (format(timestamp_sec, '.6f'))
        #print ("%.9f ", round(timestamp_sec,6))

    

    files = glob.glob(image_dir) #path.join(image_dir, "*_" + camera_name + file_extension)
    print(files[0])
    print(files[1])
    print(files[-1])
    len_files = len(files)
    print(len_files)


    #start_time = rospy.Time.now().to_sec()
    i = 0

    while (i < len_files-10): ##enumerate(files):
        img_name = files[i].split("/")[-1]
        #print (img_name)
        getcontext().prec = 6
        timestamp_sec = Decimal (int(''.join(c for c in img_name if c.isdigit())) / 1e6)
        
        #print (timestamp_sec)
        #print ("%.9f ", timestamp_sec)

        ros_stamp = rospy.Time.from_sec(timestamp_sec)
        #print("%.9f ", ros_stamp.to_sec())
        #print(ros_stamp.to_nsec())

        img = cv2.imread(files[i])
        img_msg = bridge.cv2_to_imgmsg(img, 'bgr8')
        img_msg.header.stamp = ros_stamp

        if ( i % 600 == 0  ):
            print (i)

        out_bag.write(img_topic, img_msg, ros_stamp)
        i= i+6

    out_bag.close()
    print("Finished writing to bag!")


if __name__ == "__main__":
    rospy.init_node('conversion')
    parser = argparse.ArgumentParser(description='''
    convert synthetic dataset to bag file
    ''')
    parser.add_argument('dataset_dir',
                        help='directory /img')

    args = parser.parse_args()

    synthetic_to_bags(args.dataset_dir)
