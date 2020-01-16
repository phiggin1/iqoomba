#!/usr/bin/env python
import rospy
import tf
from  geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class ExpConfig:
	def callback(self, stamped_point):
		#wait for current transform to be published		
		self.tf_listener.waitForTransform("/map", stamped_point.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
		#transform the point from whatever frame it came from to the pantil's frame
		map_pt = self.tf_listener.transformPoint("/map", stamped_point)

		marker = Marker()
		marker.header.frame_id = "/map"
		marker.type = marker.TEXT_VIEW_FACING
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

		marker.pose.position = map_pt.point
		marker.pose.position.x = marker.pose.position.x# - 0.1
		#marker.lifetime = rospy.Time.duration()
		marker.text = "OBJECT"

		'''
		print "clicked point:", stamped_point.header.frame_id, stamped_point.point
		print "transformed point:", map_pt.header.frame_id, map_pt.point
		print "marked point:", marker.header.frame_id, marker.pose.position
		'''

		self.marker_publisher.publish(marker)
		label =  raw_input("Label (enter 'skip' to try again): ") 
		if label != 'skip':
			print("writing to file")#, map_pt)
			self.f.write("%s,%s,%s,%s\n" %  (label, map_pt.point.x, map_pt.point.y, map_pt.point.z))

		
	def __init__(self):
		rospy.init_node('listener', anonymous=True)

		#start lisening for transforms
		self.tf_listener = tf.TransformListener()
		
		self.marker_publisher = rospy.Publisher('marker', Marker, queue_size=10)
		self.f = open("objects.txt", 'w')

		#subscribe to the point clicked topic and register callback method
		rospy.Subscriber("clicked_point", PointStamped, self.callback)

		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def __del__(self):
		print("closeing file")
		self.f.close()

if __name__ == '__main__':
	listener = ExpConfig()

