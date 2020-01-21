#!/usr/bin/env python3

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
from pcl_helper import float_to_rgb
from PIL import Image


class PointCloudToImages():
	def __init__(self):
		self.MIN_DIST = 0.0
		self.MAX_DIST = 4.5
		print("init")
		rospy.init_node('pc_to_image', anonymous=True)
		
		'''
		self.cam_info = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo, timeout=None)
		self.cam_model = image_geometry.PinholeCameraModel()
		self.cam_model.fromCameraInfo(self.cam_info)
		'''

		self.rgb_cam_info = rospy.wait_for_message("/rgb/camera_info", CameraInfo, timeout=None)
		self.depth_cam_info = rospy.wait_for_message("/depth/camera_info", CameraInfo, timeout=None)
		self.cam_model = image_geometry.StereoCameraModel()
		self.cam_model.fromCameraInfo(self.depth_cam_info, self.rgb_cam_info)

		self.pub = rospy.Publisher('/features', Float32MultiArray, queue_size=10)
		self.sub = rospy.Subscriber("/object", PointCloud2, self.pc_to_img_cb)
		rospy.spin()

	def pc_to_img_cb(self, cloud):
		print("cb")
		w_depth = self.depth_cam_info.width
		h_depth = self.depth_cam_info.height

		w_rgb = self.rgb_cam_info.width
		h_rgb = self.rgb_cam_info.height

		color_arr = np.zeros( (h_rgb, w_rgb, 3), np.int8)
		depth_arr = np.zeros( (h_depth, w_depth), np.int8)

		for p in pc2.read_points(cloud, skip_nans=True):
			d = math.sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2])
			r,g,b = float_to_rgb(p[3])

			left,right = self.cam_model.project3dToPixel( [ p[0], p[1], p[2] ] )
			print(left,right)
			u_depth = int(left[0])
			v_depth = int(left[1])

			u_rgb = int(right[0])
			v_rgb = int(right[1])

			color_arr[v_rgb][u_rgb][0] = r
			color_arr[v_rgb][u_rgb][1] = g
			color_arr[v_rgb][u_rgb][2] = b
			
			depth_arr[v_depth][u_depth] = ((d- self.MIN_DIST) / (self.MAX_DIST - self.MIN_DIST)) * 255


		depth_img = Image.fromarray(depth_arr, 'L')
		#depth_img.save('depth.png')
		depth_img.show()

		color_img = Image.fromarray(color_arr, 'RGB')
		#depth_img.save('depth.png')
		color_img.show()

if __name__ == '__main__':
	convert = PointCloudToImages()
