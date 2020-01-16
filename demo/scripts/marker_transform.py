#!/usr/bin/env python

import rospy
import tf
from  geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def get_stamped_point(x,y,z, frame_id):
	p = PointStamped()
	p.header.frame_id = frame_id
	p.header.stamp = rospy.Time.now()
	p.point.x = x
	p.point.y = y
	p.point.z = z

	return p

class MarkerTransform:
	def __init__(self):
		print("init listener")
		rospy.init_node('marker_transform', anonymous=True)
		self.tf_listener = tf.TransformListener()
		self.sub = rospy.Subscriber("/object_markers", MarkerArray, self.marker_cb)
		self.obj_markers_pub = rospy.Publisher('/object_markers_map', MarkerArray, queue_size=10)
		rospy.spin()

	def marker_cb(self, marker_array):
		print("cb")
		for marker in marker_array.markers:
			p = get_stamped_point(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, marker.header.frame_id)

			self.tf_listener.waitForTransform(marker.header.frame_id, "/map", rospy.Time.now(), rospy.Duration(5.0))
			#transform the point from whatever frame it came from to the pantil's frame
			map_pt = self.tf_listener.transformPoint("/map", p)

			marker.header.frame_id = "/map"
			marker.pose.position.x = map_pt.point.x
			marker.pose.position.y = map_pt.point.y
			marker.pose.position.z = map_pt.point.z

		self.obj_markers_pub.publish(marker_array)


if __name__ == '__main__':
	try:
		marker_transform = MarkerTransform()
	except rospy.ROSInterruptException:
		pass

