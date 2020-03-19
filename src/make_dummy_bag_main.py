#!/usr/bin/env python2

# IMPORTS
# system
import os, sys, argparse, time
import pdb
# math
import numpy as np
# ros
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped, Twist, Pose
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def run_execution_loop():
    rate = rospy.Rate(100)
    print("in main loop")
    while not rospy.is_shutdown():
        rate.sleep()
    

if __name__ == '__main__':
    np.set_printoptions(linewidth=160, suppress=True)  # format numpy so printing matrices is more clear
    print("Starting make dummy bag main [running python {}]".format(sys.version_info[0]))
    rospy.init_node('make_dummy_bag_node', anonymous=True)
    run_execution_loop()
    print("--------------- FINISHED ---------------")

