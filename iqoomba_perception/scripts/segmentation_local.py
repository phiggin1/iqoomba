#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import struct
import ctypes
import random
import time
import math

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from pcl_helper import ros_to_pcl, pcl_to_ros, rgb_to_float, float_to_rgb

from pc_to_color_depth import PointCloudToImages
from fuse_images import fuse_color_depth
from featureize import get_features

THRESHOLD  = 0.005

def distance_point_to_plane(p, model):
    return ( model[0]*p[0] + model[1]*p[1] + model[2]*p[2] + model[3] );

def marker2str(marker):
	return str(format(marker.pose.position.x, '.3f')) + " " + str(format(marker.pose.position.y, '.3f')) + " " + str(format(marker.pose.position.z, '.3f'))

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

def dist(a, b):
	ax = a.pose.position.x
	ay = a.pose.position.y
	az = a.pose.position.z
	
	bx = b.pose.position.x
	by = b.pose.position.y
	bz = b.pose.position.z
	
	return math.sqrt( (ax-bx)**2 + (ay-by)**2 + (az-bz)**2 )

def find_closest(old_obj, new_objects):
	min_dist = 999.9
	min_indx = 0
	for i, new_obj in enumerate(new_objects):
		d = dist(old_obj, new_obj)
		if  d < min_dist:
			min_dist = d
			min_indx = i

	return min_indx, min_dist

class Segmentation:
	def get_clusters(self, cloud):
		#print("point cloud size = ",len(cloud.data), " bytes")

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
		ec.set_MinClusterSize (200)
		ec.set_MaxClusterSize (5000)
		ec.set_SearchMethod (tree)
		cluster_indices = ec.Extract()
		t2 =  time.clock()

		#print("ECE took ", t2-t1)

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

			pc_msg = pcl_to_ros(obj, cloud.header)
			self.pub.publish(pc_msg)

		self.track(marker_array)

	def track(self, marker_array):
		if self.objects == None:
			self.objects = []
			for marker in marker_array.markers:
				self.objects.append(marker)
			print(len(self.objects), " initial objects")
		else:
			old_objects = self.objects
			new_objects = marker_array.markers
			
			old_indxs = list(range(len(old_objects)))
			new_indxs = list(range(len(new_objects)))

			if len(old_indxs) >= len(new_indxs):
				for i, old in enumerate(old_objects):
					closest_indx, min_dist = find_closest(old, new_objects)
					#print(i, marker2str(old), closest_indx, marker2str(new_objects[closest_indx]), format(min_dist, '.3f'))
					if min_dist < THRESHOLD:
						if i in old_indxs: 
							old_indxs.remove(i)
						if closest_indx in new_indxs: 
							new_indxs.remove(closest_indx)
			else:
				for i, new in enumerate(new_objects):
					closest_indx, min_dist = find_closest(new, old_objects)
					#print(i, marker2str(new), closest_indx, marker2str(old_objects[closest_indx]), format(min_dist, '.3f'))
					if min_dist < THRESHOLD:
						if i in new_indxs: 
							new_indxs.remove(i)
						if closest_indx in old_indxs: 
							old_indxs.remove(closest_indx)

			if len(old_indxs) > len(new_indxs):
				print("things removed")
				#TODO Handle in future
			elif len(old_indxs) < len(new_indxs):
				print("things added")
				#TODO Handle in future
			
			if len(old_indxs) == len(new_indxs) and len(old_indxs) > 0:
				print("things have moved")
				print(old_indxs, new_indxs)
				for old_indx in old_indxs:
					new = [new_objects[i] for i in new_indxs]
					closest_indx, min_dist = find_closest(self.objects[old_indx], new)
	
					self.objects[old_indx].pose.position.x = new[closest_indx].pose.position.x
					self.objects[old_indx].pose.position.y = new[closest_indx].pose.position.y
					self.objects[old_indx].pose.position.z = new[closest_indx].pose.position.z

			for obj in self.objects:
				obj.header.seq = obj.header.seq + 1
				obj.header.stamp = rospy.Time.now()

			self.tracked_objects.publish(self.objects)

	def __init__(self):
		rospy.init_node('ransac_filter', anonymous=True)
		self.objects = None
		self.sub = rospy.Subscriber("/objects_filtered", PointCloud2, self.get_clusters)
		self.pub = rospy.Publisher('/object', PointCloud2, queue_size=10)
		self.tracked_objects = rospy.Publisher('/tracked_objects', MarkerArray, queue_size=10)
		rospy.spin()

if __name__ == '__main__':
	segment = Segmentation()
