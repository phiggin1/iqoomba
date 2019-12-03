#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import struct
import ctypes
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def float_to_rgb(float_rgb):
    """ Converts a packed float RGB format to an RGB list
        Args:
            float_rgb: RGB value packed as a float
        Returns:
            color (list): 3-element list of integers [0-255,0-255,0-255]
    """
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value

    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    color = [r,g,b]

    return color

def ros_to_pcl(ros_cloud):
	""" Converts a ROS PointCloud2 message to a pcl PointXYZRGB
    
	Args:
		ros_cloud (PointCloud2): ROS PointCloud2 message
            
        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
	"""
	points_list = []
	cnt = 0
	for data in pc2.read_points(ros_cloud, skip_nans=True):
		points_list.append([data[0], data[1], data[2], data[3]])
		print(data[0], data[1], data[2], data[3], float_to_rgb(data[3]))
		cnt = cnt + 1

	print(cnt)
	pcl_data = pcl.PointCloud_PointXYZRGB()
	pcl_data.from_list(points_list)

	return pcl_data 

def pointcloud_cb(cloud):
	p = ros_to_pcl(cloud)

def main():
	print("init listener")
	rospy.init_node('test', anonymous=True)
	sub = rospy.Subscriber("/object", PointCloud2, pointcloud_cb)
	rospy.spin()

if __name__ == '__main__':
	main()
