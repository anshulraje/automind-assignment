#!/usr/bin/env python3

import rospy
from  ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from sensor_msgs.msg import PointCloud2
from numpy import ndarray

class ObjectDetection:

    def __init__(self):
        rospy.init_node("object_detection", anonymous=True)

        rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, callback=self.pointcloud_callback)

        self.cloud_in = PointCloud2
        self.cloud_out = 0

    def pointcloud_callback(self, data):
        self.cloud_out = pointcloud2_to_xyz_array(data, remove_nans=True)
        print(self.cloud_out)

def main():
    obj = ObjectDetection()

    while not rospy.is_shutdown():
        continue

if __name__ == "__main__":
    main()