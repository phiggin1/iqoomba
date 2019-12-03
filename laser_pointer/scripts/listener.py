#!/usr/bin/env python
import rospy
import tf
from  geometry_msgs.msg import PointStamped
import zmq

class PanTiltListener:
	def callback(self, stamped_point):
		#wait for current transform to be published		
		self.tf_listener.waitForTransform("/pantilt_link", stamped_point.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
		#transform the point from whatever frame it came from to the pantil's frame
		pan_tilt_pt = self.tf_listener.transformPoint("/pantilt_link", stamped_point)

		print "clicked point:", stamped_point.header.frame_id, stamped_point.point
		print "transformed point:", pan_tilt_pt.header.frame_id, pan_tilt_pt.point

		#send a message to the pi with transformed coords
		self.socket.send("%s %s %s" % (pan_tilt_pt.point.x, pan_tilt_pt.point.y, pan_tilt_pt.point.z))
		
	def __init__(self):
		# In ROS, nodes are uniquely named. If two nodes with the same
		# name are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
		rospy.init_node('listener', anonymous=True)

		#start lisening for transforms
		self.tf_listener = tf.TransformListener()


		#TODO: need to generalize pi address and port
		context = zmq.Context()
		self.socket = context.socket(zmq.PAIR)
		self.socket.bind("tcp://*:%s" % 4444)

		print("bound port")

		#subscribe to the point clicked topic and register callback method
		rospy.Subscriber("clicked_point", PointStamped, self.callback)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

if __name__ == '__main__':
    #TODO grab ip address and port from command line arguments
	listener = PanTiltListener()
