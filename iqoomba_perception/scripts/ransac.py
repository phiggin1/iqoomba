#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import struct
import ctypes
import random
import time
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from pcl_helper import ros_to_pcl, pcl_to_ros, rgb_to_float, float_to_rgb


def distance_point_to_plane(p, model):
    return ( model[0]*p[0] + model[1]*p[1] + model[2]*p[2] + model[3] );


class RansacFilter:
	def pointcloud_cb(self, cloud):
		filtered_cloud = self.ground_filter(cloud)
		pc_msg = pcl_to_ros(filtered_cloud, cloud.header)

		self.pub.publish(pc_msg)


	def ground_filter(self, cloud):
		pcl_cloud = ros_to_pcl(cloud)

		t1 =  time.clock()
		'''
		seg = pcl_cloud.make_segmenter_normals(ksearch=5)
		seg.set_optimize_coefficients(True)
		seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
		seg.set_normal_distance_weight(0.1)
		seg.set_method_type(pcl.SAC_RANSAC)
		seg.set_max_iterations(10)
		seg.set_distance_threshold(0.03)
		'''

		seg = pcl_cloud.make_segmenter()
		seg.set_optimize_coefficients(True)
		seg.set_model_type(pcl.SACMODEL_PLANE)
		seg.set_method_type(pcl.SAC_RANSAC)
		seg.set_distance_threshold(0.03)


		indices, model = seg.segment()

		cnt = 0
		i = 0
		ind = []
		for p in pc2.read_points(cloud, skip_nans=True):
			d = distance_point_to_plane(p, model)
			#0.005 is a fudge factor to correct for any error in the model
			#to avoid having any parts of the ground to creep in
			if d > 0.005:
				cnt = cnt + 1
				ind.append(i)

			i = i + 1
		t2 =  time.clock()
		print("RANSAC took ", t2-t1)

		print("model:", model)
		print("org size", cloud.width*cloud.height, "ransac size", cnt)

		return pcl_cloud.extract(ind, negative=False)

	def __init__(self):
		rospy.init_node('ransac_filter', anonymous=True)
		self.pub = rospy.Publisher('/objects', PointCloud2, queue_size=10)
		self.sub = rospy.Subscriber("/filtered", PointCloud2, self.pointcloud_cb)
		rospy.spin()

if __name__ == '__main__':
	filter = RansacFilter()
