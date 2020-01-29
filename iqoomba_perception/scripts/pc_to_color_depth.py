#!/usr/bin/env python

import numpy as np
import struct
import ctypes
import rospy
import image_geometry
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
from PIL import Image
from pcl_helper import float_to_rgb

MIN_DIST = 0.0
MAX_DIST = 4.5

class PointCloudToImages():
	def __init__(self, data_path="~/data/"):
		'''
		self.cam_info = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo, timeout=None)
		self.cam_model = image_geometry.PinholeCameraModel()
		self.cam_model.fromCameraInfo(self.cam_info)
		'''
		self.data_path = data_path
		self.rgb_cam_info = rospy.wait_for_message("/rgb_to_depth/camera_info", CameraInfo, timeout=None)
		self.depth_cam_info = rospy.wait_for_message("/depth/camera_info", CameraInfo, timeout=None)
		self.cam_model = image_geometry.StereoCameraModel()
		self.cam_model.fromCameraInfo(self.depth_cam_info, self.rgb_cam_info)



	def convert(self, cloud):
		rgb_path = self.data_path+"rgb/rgb.png"
		depth_path = self.data_path+"depth/depth.png"
		#print(self.depth_cam_info, self.rgb_cam_info)
		w_depth = self.depth_cam_info.width
		h_depth = self.depth_cam_info.height

		w_rgb = self.rgb_cam_info.width
		h_rgb = self.rgb_cam_info.height

		print(w_depth, h_depth, w_rgb, h_rgb)

		color_arr = np.zeros( (h_rgb, w_rgb, 3), np.int8)
		depth_arr = np.zeros( (h_depth, w_depth), np.int8)

		for p in cloud:
			d = math.sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2])
			r,g,b = float_to_rgb(p[3])

			left,right = self.cam_model.project3dToPixel( [ p[0], p[1], p[2] ] )
			#print(left,right)
			u_depth = int(left[0])
			v_depth = int(left[1])

			u_rgb = int(right[0])
			v_rgb = int(right[1])

			color_arr[v_rgb][u_rgb][0] = r
			color_arr[v_rgb][u_rgb][1] = g
			color_arr[v_rgb][u_rgb][2] = b
			
			depth_arr[v_depth][u_depth] = ((d- MIN_DIST) / (MAX_DIST - MIN_DIST)) * 255

		depth_img = Image.fromarray(depth_arr, 'L')
		depth_img.save(depth_path)
		#depth_img.show()

		color_img = Image.fromarray(color_arr, 'RGB')
		depth_img.save(rgb_path)
		#color_img.show()
		print("Wrote depth and color to ",rgb_path," and ",depth_path)


