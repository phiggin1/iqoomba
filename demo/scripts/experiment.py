#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from sound_play.libsoundplay import SoundClient

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


import random
import time

def laser_base_pos():
	stamped_point = PointStamped()
	stamped_point.header.frame_id = "pantilt_link"
	stamped_point.header.stamp = rospy.Time.now()
	stamped_point.point.x = 0.07
	stamped_point.point.y = 0.0
	stamped_point.point.z = 0.0

	return stamped_point

def get_stamped_point(x,y,z):
	stamped_point = PointStamped()
	stamped_point.header.frame_id = "map"
	stamped_point.header.stamp = rospy.Time.now()
	stamped_point.point.x = x
	stamped_point.point.y = y
	stamped_point.point.z = z

	return stamped_point


#get yaw heading from a point
def get_move_base_goal(x,y,z):
	goal = MoveBaseGoal()

	goal.target_pose.header.frame_id = "map"
	
	#get yaw from x,y

	yaw = 0.0
	goal.target_pose.pose.orientation = Quaternion(*np.asarray(quaternion_from_euler(0.0, 0.0, yaw)))

	goal.target_pose.header.stamp = rospy.Time.now()
	return goal

def get_objects(fname):
	objects = []
	f = open(fname, 'r')
	for line in f:
		x,y,z = line.split(',')
		objects.append( ( float(x), float(y), float(z) ) )

	return objects
		
def get_questions(fname):
	questions = []
	f = open(fname, 'r')
	for line in f:
		questions.append(line)

	return questions

if __name__ == '__main__':
	rospy.init_node('experiment', anonymous = True)
	soundhandle = SoundClient()
	voice = 'voice_kal_diphone'
	volume = 1.0
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	point_publisher = rospy.Publisher('clicked_point', PointStamped, queue_size=10)
	rospy.sleep(1)

	print("waiting for action server")
	client.wait_for_server()

	questions = get_questions('/home/phiggins/catkin_ws/src/demo/scripts/questions.txt')
	objects = get_objects('/home/phiggins/catkin_ws/src/demo/scripts/objects.txt')

	'''
	client.send_goal(get_move_base_goal(x,y,z))
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	'''
	point_publisher.publish(laser_base_pos())
	time.sleep(5.0)

	for i in range(len(objects)):
		x = objects[i][0]
		y = objects[i][1]
		z = objects[i][2]
		rand_quest = random.randint(0,1)
		q = questions[rand_quest]
		print("exp", x,y,z, q)

		point_publisher.publish(get_stamped_point(x,y,z))

		#soundhandle.say(q, voice, volume)

		time.sleep(5.0)

	point_publisher.publish(laser_base_pos())
