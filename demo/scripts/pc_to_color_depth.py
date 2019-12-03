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

def float_to_rgb(float_rgb):
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value

    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    color = [r,g,b]

    return color


class PointCloudToImages():
	def __init__(self):
		self.MIN_DIST = 0.0
		self.MAX_DIST = 4.5
		print("init")
		rospy.init_node('pc_to_image', anonymous=True)
		self.cam_info = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo, timeout=None)

		self.cam_model = image_geometry.PinholeCameraModel()
		self.cam_model.fromCameraInfo(self.cam_info)

		self.pub = rospy.Publisher('/features', Float32MultiArray, queue_size=10)
		self.sub = rospy.Subscriber("/object", PointCloud2, self.pc_to_img_cb)
		rospy.spin()

	def pc_to_img_cb(self, cloud):
		w = self.cam_info.width
		h = self.cam_info.height

		color_arr = np.zeros( (h, w, 3), np.int8)
		depth_arr = np.zeros( (h, w), np.int8)

		for p in pc2.read_points(cloud, skip_nans=True):
			d = math.sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2])
			r,g,b = float_to_rgb(p[3])

			u,v = self.cam_model.project3dToPixel( [ p[0], p[1], p[2] ] )
			u = int(u)
			v = int(v)

			color_arr[v][u][0] = r
			color_arr[v][u][1] = g
			color_arr[v][u][2] = b
			
			depth_arr[v][u] = ((d- self.MIN_DIST) / (self.MAX_DIST - self.MIN_DIST)) * 255


		depth_img = Image.fromarray(depth_arr, 'L')
		#depth_img.save('depth.png')
		depth_img.show()

		color_img = Image.fromarray(color_arr, 'RGB')
		#depth_img.save('depth.png')
		color_img.show()

if __name__ == '__main__':
	convert = PointCloudToImages()
