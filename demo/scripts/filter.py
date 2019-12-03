#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import struct
import ctypes
import random
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def distance_point_to_plane(p, model):
    return ( model[0]*p[0] + model[1]*p[1] + model[2]*p[2] + model[3] );


#pcl_helper.py
def ros_to_pcl(ros_cloud):
	points_list = []

	for data in pc2.read_points(ros_cloud, skip_nans=True):
		points_list.append([data[0], data[1], data[2], data[3]])

	pcl_data = pcl.PointCloud_PointXYZRGB()
	pcl_data.from_list(points_list)

	return pcl_data 

def pcl_to_ros(pcl_array, stamp, frame_id, seq):
	ros_msg = PointCloud2()

	ros_msg.header.stamp = stamp
	ros_msg.header.frame_id = frame_id
	ros_msg.header.seq = seq

	ros_msg.height = 1
	ros_msg.width = pcl_array.size

	ros_msg.fields.append(PointField(name="x", offset=0,
                            datatype=PointField.FLOAT32, count=1))
	ros_msg.fields.append(PointField(name="y", offset=4,
                            datatype=PointField.FLOAT32, count=1))
	ros_msg.fields.append(PointField(name="z", offset=8,
                            datatype=PointField.FLOAT32, count=1))
	ros_msg.fields.append(PointField(name="rgb", offset=16,
                            datatype=PointField.FLOAT32, count=1))

	ros_msg.is_bigendian = False
	ros_msg.point_step = 32
	ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
	ros_msg.is_dense = False

	buffer = []
	msg_data = []
	for data in pcl_array:
		s = struct.pack('>f', data[3])
		i = struct.unpack('>l', s)[0]
		pack = ctypes.c_uint32(i).value

		r = (pack & 0x00FF0000) >> 16
		g = (pack & 0x0000FF00) >> 8
		b = (pack & 0x000000FF)

		buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

	for line in buffer:
		for b in line:
			msg_data.append(ctypes.c_uint8(b).value)

	ros_msg.data = msg_data	

	return ros_msg

def rgb_to_float(color):
    hex_r = (0xff & color[0]) << 16
    hex_g = (0xff & color[1]) << 8
    hex_b = (0xff & color[2])

    hex_rgb = hex_r | hex_g | hex_b

    float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

    return float_rgb


def float_to_rgb(float_rgb):
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value

    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    color = [r,g,b]

    return color

def gound_filer(cloud):
	pcl_cloud = ros_to_pcl(cloud)
	seg = pcl_cloud.make_segmenter_normals(ksearch=50)
	seg.set_optimize_coefficients(True)
	seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
	seg.set_normal_distance_weight(0.1)
	seg.set_method_type(pcl.SAC_RANSAC)
	seg.set_max_iterations(100)
	seg.set_distance_threshold(0.03)
	indices, model = seg.segment()

	cnt = 0
	i = 0
	ind = []
	for p in pc2.read_points(cloud, skip_nans=True):
		d = distance_point_to_plane(p, model)

		if d > 0.005:
			cnt = cnt + 1
			ind.append(i)

		i = i + 1

	#print("model:", model)
	#print("org size", cloud.width*cloud.height, "ransac size", cnt)

	return pcl_cloud.extract(ind, negative=False)

def get_larget_cluster(pcl_cloud):
	#convert pointcloud_xyzrgb to pointcloud_xyz since pointcloud_xyzrgb
	#	does not support make_EuclideanClusterExtraction
	xyz_pc = pcl.PointCloud()
	xyz_pc.from_list( [ x[:3] for x in pcl_cloud.to_list()] )

	tree = xyz_pc.make_kdtree()

	ec = xyz_pc.make_EuclideanClusterExtraction()
	ec.set_ClusterTolerance (0.02)
	ec.set_MinClusterSize (100)
	ec.set_MaxClusterSize (25000)
	ec.set_SearchMethod (tree)
	cluster_indices = ec.Extract()

	cloud_cluster = pcl.PointCloud_PointXYZRGB()

	indx = []
	#for all j clusters
	for j, indices in enumerate(cluster_indices):
		tmp_indx = []

		#for all points in the cluster j
		for i, indice in enumerate(indices):
			tmp_indx.append(indice)

		if ( len(tmp_indx) > len(indx) ):
			indx = tmp_indx

	return pcl_cloud.extract(indx, negative=False)

class RansacFilter:
	def pointcloud_cb(self, cloud):
		
		filtered_cloud = gound_filer(cloud)

		cloud_cluster = get_larget_cluster(filtered_cloud)

		pc_msg = pcl_to_ros(cloud_cluster, stamp=cloud.header.stamp, frame_id=cloud.header.frame_id, seq=cloud.header.seq)

		self.pub.publish(pc_msg)

	def __init__(self):
		print("init listener")
		rospy.init_node('ransac_filter', anonymous=True)
		self.pub = rospy.Publisher('/object', PointCloud2, queue_size=10)
		self.sub = rospy.Subscriber("/filtered", PointCloud2, self.pointcloud_cb)
		rospy.spin()

if __name__ == '__main__':
	filter = RansacFilter()
