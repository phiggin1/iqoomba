#!/usr/bin/env python

import rospy
import tf
from  geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import os

def get_marker(i, label, x, y, z):
	marker = Marker()

	marker.id = i
	marker.header.frame_id = "/map"
	marker.header.stamp = rospy.Time.now()

	marker.type = marker.TEXT_VIEW_FACING
	marker.text = label

	marker.action = marker.ADD

	marker.scale.z = 0.1

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

def send_markers():
	marker_publisher = rospy.Publisher('markers', MarkerArray, queue_size=10)
	rospy.init_node('publish_markers', anonymous=True)

	marker_array = MarkerArray()

	f = open("/home/iral/objects.csv", 'r')
	count = 0
	for line in f:
		label,x,y,z = line.split(',')
		marker_array.markers.append(get_marker(int(count), label, float(x), float(y) ,float(z)))
		count += 1
	f.close()

	rate = rospy.Rate(0.1)
	while not rospy.is_shutdown():
		marker_publisher.publish(marker_array)
		rate.sleep()

if __name__ == '__main__':
	try:
		send_markers()
	except rospy.ROSInterruptException:
		pass

