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
# from geometry_msgs.msg import PoseStamped, Twist, Pose
from sensor_msgs.msg import Imu, MultiEchoLaserScan
# from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class make_dummy_bag:
    def __init__(self):
        rospy.init_node('make_dummy_bag_node', anonymous=True)
        self.ns = rospy.get_param('~ns')
        self.num_bots = rospy.get_param('~num_bots')
        # rospy.Subscriber('/horizontal_laser_2d', MultiEchoLaserScan, self.h_lidar_cb, queue_size=10)  # optitrack pose
        # rospy.Subscriber('/vertical_laser_2d', MultiEchoLaserScan, self.v_lidar_cb, queue_size=10)  # optitrack pose
        rospy.Subscriber('/imu', Imu, self.imu_cb, queue_size=10)  # optitrack pose

        self.out_bag = rosbag.Bag('/bags/output_bags/test_new.bag', 'w')


    # def h_lidar_cb(self, msg):
    #     for i in range(self.num_bots):
    #         self.out_bag.write("{}{}/horizontal_laser_2d".format(self.ns, i+1), msg)
        

    # def v_lidar_cb(self, msg):
    #     for i in range(self.num_bots):
    #         self.out_bag.write("{}{}/vertical_laser_2d".format(self.ns, i+1), msg)

    def imu_cb(self, msg):
        print(msg)
        for i in range(self.num_bots):
            self.out_bag.write("/{}{}/imu".format(self.ns, i+1), msg)

    
    def run(self):
        rate = rospy.Rate(100)
        print("in main loop")
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    np.set_printoptions(linewidth=160, suppress=True)  # format numpy so printing matrices is more clear
    print("Starting make dummy bag main [running python {}]".format(sys.version_info[0]))

    try:
        program = make_dummy_bag()
        program.run()
        program.out_bag.close()
    except:
        import traceback
        traceback.print_exc()


    print("--------------- FINISHED ---------------")

