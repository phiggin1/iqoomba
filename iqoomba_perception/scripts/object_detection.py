#!/usr/bin/env python3

import rospy
import pcl
import time
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from pcl_helper import ros_to_pcl, pcl_to_ros

THRESHOLD  = 0.0125

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

def marker2str(marker):
	return str(format(marker.pose.position.x, '.3f')) + " " + str(format(marker.pose.position.y, '.3f')) + " " + str(format(marker.pose.position.z, '.3f'))

def distance_point_to_plane(p, model):
    return ( model[0]*p[0] + model[1]*p[1] + model[2]*p[2] + model[3] );

def get_marker(i, frame_id, obj):
	marker = Marker()

	marker.id = i
	marker.header.frame_id = frame_id
	marker.header.stamp = rospy.Time.now()

	marker.type = marker.TEXT_VIEW_FACING
	marker.text = "obj_" + str(i)

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

	marker.pose.position.x = float(obj[0])
	marker.pose.position.y = float(obj[1])
	marker.pose.position.z = float(obj[2])

	return marker

def ground_filter(cloud):
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
	seg.set_optimize_coefficients(False)
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	seg.set_distance_threshold(0.006)

	indices, model = seg.segment()

	'''
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
	'''
	t2 =  time.clock()
	print("RANSAC took ", t2-t1)
	
	print("model:", model)
	print("org size", cloud.width*cloud.height, "ransac size", cloud.width*cloud.height-len(indices))


	return pcl_cloud.extract(indices, negative=True)

def get_clusters(pcl_cloud):
	#pcl_cloud = ros_to_pcl(cloud)
	
	#convert pointcloud_xyzrgb to pointcloud_xyz 
	# since pointcloud_xyzrgb does not support make_EuclideanClusterExtraction
	xyz_pc = pcl.PointCloud()
	xyz_pc.from_list( [ x[:3] for x in pcl_cloud.to_list() ] )

	tree = xyz_pc.make_kdtree()

	ec = xyz_pc.make_EuclideanClusterExtraction()
	ec.set_ClusterTolerance (0.0025)
	ec.set_MinClusterSize (10000)
	ec.set_MaxClusterSize (35000)
	ec.set_SearchMethod (tree)
	return ec.Extract()


class ObjectDetection():
	def __init__(self):
		self.objects = None
		rospy.init_node('object_detection', anonymous=True)

		self.objects_pub = rospy.Publisher("/objects", PointCloud2, queue_size=10)
		self.object_pub = rospy.Publisher('/object', PointCloud2, queue_size=10)
		#self.obj_markers_pub = rospy.Publisher('/object_markers', MarkerArray, queue_size=10)
		self.tracked_objects = rospy.Publisher("tracked_objects", MarkerArray, queue_size=10)

		self.sub = rospy.Subscriber("/filtered", PointCloud2, self.pointcloud_cb)
		rospy.spin()

	def pointcloud_cb(self, ros_cloud):
		ransac_start =  time.clock()
		#filtered_cloud = ground_filter(ros_cloud)
		pcl_cloud = ros_to_pcl(ros_cloud)

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
		seg.set_optimize_coefficients(False)
		seg.set_model_type(pcl.SACMODEL_PLANE)
		seg.set_method_type(pcl.SAC_RANSAC)
		seg.set_distance_threshold(0.006)

		indices, model = seg.segment()

		'''
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
		'''
		t2 =  time.clock()
		#print("RANSAC took ", t2-t1)
		
		#print("model:", model)
		#print("org size", ros_cloud.width*ros_cloud.height, "ransac size", ros_cloud.width*ros_cloud.height-len(indices))


		filtered_cloud = pcl_cloud.extract(indices, negative=True)
		ransac_end =  time.clock()
		pc_msg = pcl_to_ros(filtered_cloud, ros_cloud.header)
		self.objects_pub.publish(pc_msg)
		
		ece_start =  time.clock()
		#cluster_indices = get_clusters(filtered_cloud)
		xyz_pc = pcl.PointCloud()
		xyz_pc.from_list( [ x[:3] for x in filtered_cloud.to_list() ] )

		tree = xyz_pc.make_kdtree()

		ec = xyz_pc.make_EuclideanClusterExtraction()
		ec.set_ClusterTolerance (0.0025)
		ec.set_MinClusterSize (10000)
		ec.set_MaxClusterSize (35000)
		ec.set_SearchMethod (tree)
		cluster_indices = ec.Extract()
		ece_end =  time.clock()

		
		tracking_start =  time.clock()
		objects = []
		#for all j clusters
		#print("# clusters ", len(cluster_indices))
		for j, indices in enumerate(cluster_indices):
			#print("Cluster", j ,": # points ", len(indices))
			sum_x = 0
			sum_y = 0
			sum_z = 0
			#for all points in the cluster j
			for i, indice in enumerate(indices):
				sum_x = sum_x + filtered_cloud[indice][0]
				sum_y = sum_y + filtered_cloud[indice][1]
				sum_z = sum_z + filtered_cloud[indice][2]

			x = sum_x/len(indices)
			y = sum_y/len(indices)
			z = sum_z/len(indices)

			objects.append([x,y,z])


			obj = filtered_cloud.extract(indices, negative=False)
			pc_msg = pcl_to_ros(obj, ros_cloud.header)
			self.object_pub.publish(pc_msg)
			#print("object at ", x,y,z)# "\nwith features: ",features)

		marker_array = MarkerArray()
		for i, obj in enumerate(objects):
			marker_array.markers.append(get_marker(i, ros_cloud.header.frame_id, obj))

		self.track(marker_array)
		tracking_end =  time.clock()

		print("RANSAC: ", ransac_end-ransac_start)
		print("ECE: ", ece_end-ece_start)
		print("Tracking: ", tracking_end-tracking_start)
		print("Total: ", tracking_end-ransac_start)

		#self.obj_markers_pub.publish(marker_array)

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
						old_indxs.remove(i)
						new_indxs.remove(closest_indx)
			else:
				for i, new in enumerate(new_objects):
					closest_indx, min_dist = find_closest(new, old_objects)
					#print(i, marker2str(new), closest_indx, marker2str(old_objects[closest_indx]), format(min_dist, '.3f'))
					if min_dist < THRESHOLD:
						new_indxs.remove(i)
						old_indxs.remove(closest_indx)

			#print(old_indxs, new_indxs)
			if len(old_indxs) > len(new_indxs):
				print("things removed")
				#TODO Handle in future
			elif len(old_indxs) < len(new_indxs):
				print("things added")
				#TODO Handle in future
			
			if len(old_indxs) == len(new_indxs) and len(old_indxs) > 0:
				print("things have moved")
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
if __name__ == '__main__':
	detector = ObjectDetection()
