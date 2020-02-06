#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import struct
import ctypes
import random
import time


import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from pcl_helper import ros_to_pcl, pcl_to_ros, rgb_to_float, float_to_rgb

from pc_to_color_depth import PointCloudToImages
from fuse_images import fuse_color_depth
from featureize import get_features

def distance_point_to_plane(p, model):
    return ( model[0]*p[0] + model[1]*p[1] + model[2]*p[2] + model[3] );


def get_stamped_point(x,y,z, frame_id):
	p = PointStamped()
	p.header.frame_id = frame_id
	p.header.stamp = rospy.Time.now()
	p.point.x = x
	p.point.y = y
	p.point.z = z

	return p

def get_marker(i, label, frame_id, x, y, z):
	marker = Marker()

	marker.id = i
	marker.header.frame_id = frame_id
	marker.header.stamp = rospy.Time.now()

	marker.type = marker.TEXT_VIEW_FACING
	marker.text = label

	marker.action = marker.ADD

	marker.scale.z = 0.025

	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 1.0		
	marker.color.a = 1.0

	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0

	marker.pose.position.x = float(x)
	marker.pose.position.y = float(y)
	marker.pose.position.z = float(z)

	return marker

class Segmentation:
	def get_clusters(self, cloud):

		print("point cloud size = ",len(cloud.data), " bytes")

		marker_array = MarkerArray()

		pcl_cloud = ros_to_pcl(cloud)
		#convert pointcloud_xyzrgb to pointcloud_xyz 
		# since pointcloud_xyzrgb does not support make_EuclideanClusterExtraction
		xyz_pc = pcl.PointCloud()
		xyz_pc.from_list( [ x[:3] for x in pcl_cloud.to_list() ] )
		tree = xyz_pc.make_kdtree()
	
		t1 =  time.clock()
		ec = xyz_pc.make_EuclideanClusterExtraction()
		ec.set_ClusterTolerance (0.005)
		ec.set_MinClusterSize (1000)
		ec.set_MaxClusterSize (20000)
		ec.set_SearchMethod (tree)
		cluster_indices = ec.Extract()
		t2 =  time.clock()

		print("ECE took ", t2-t1)

		#for all j clusters
		print("# clusters ", len(cluster_indices))
		for j, indices in enumerate(cluster_indices):
			print("Cluster", j ,": # points ", len(indices))

			sum_x = 0
			sum_y = 0
			sum_z = 0
			#for all points in the cluster j
			for i, indice in enumerate(indices):
				sum_x = sum_x + xyz_pc[indice][0]
				sum_y = sum_y + xyz_pc[indice][1]
				sum_z = sum_z + xyz_pc[indice][2]


			x = sum_x/len(indices)
			y = sum_y/len(indices)
			z = sum_z/len(indices)

			label = "obj_" + str(j)
			marker_array.markers.append(get_marker(j, label, cloud.header.frame_id, x, y, z))

			obj = pcl_cloud.extract(indices, negative=False)

			pc_msg = pcl_to_ros(obj, stamp=cloud.header.stamp, frame_id=cloud.header.frame_id, seq=cloud.header.seq)
			self.pub.publish(pc_msg)

		self.obj_markers_pub.publish(marker_array)

	def __init__(self):
		rospy.init_node('ransac_filter', anonymous=True)
		self.sub = rospy.Subscriber("/objects", PointCloud2, self.get_clusters)
		self.pub = rospy.Publisher('/object', PointCloud2, queue_size=10)
		self.obj_markers_pub = rospy.Publisher('/object_markers', MarkerArray, queue_size=10)
		rospy.spin()

if __name__ == '__main__':
	segment = Segmentation()
