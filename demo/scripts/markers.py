#!/usr/bin/env python
import rospy
import tf
from  geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import os

def get_marker(i, x, y, z):
	marker = Marker()

	marker.id = i
	marker.header.frame_id = "/map"
	marker.header.stamp = rospy.Time.now()

	marker.type = marker.TEXT_VIEW_FACING
	marker.text = "object"+str(i)

	marker.action = marker.ADD

	marker.scale.x = 0.1
	marker.scale.y = 0.1
	marker.scale.z = 0.1

	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 1.0		
	marker.color.a = 1.0

	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0

	marker.pose.position.x = float(x)#-0.1
	marker.pose.position.y = float(y)
	marker.pose.position.z = float(z)

	return marker

def send_markers():
	marker_publisher = rospy.Publisher('markers', MarkerArray, queue_size=10)
	rospy.init_node('publish_markers', anonymous=True)

	marker_array = MarkerArray()

	print(os.getcwd())

	count = 0
	f = open("/home/phiggins/catkin_ws/src/demo/scripts/objects.txt", 'r')
	for line in f:
		x,y,z = line.split(',')
		print("marker", x,y,z)
		marker_array.markers.append(get_marker(count, x, y ,z))
		count+=1
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

