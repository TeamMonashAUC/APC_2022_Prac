#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data)
    print(type(gen))
    for p in gen:
        print(p) 

def listener():
    rospy.init_node('pylistener', anonymous=True)
    rospy.Subscriber('airsim_node/base_link/lidar/Lidar1', PointCloud2, callback_pointcloud)
    rospy.spin()

if __name__ == '__main__':

    listener()