#!/usr/bin/env python

import time
import sys
import os
import rospy
import actionlib
import tf
import math
import zmq
import numpy as np

from copy import copy

from sound_play.libsoundplay import SoundClient

from tts.msg import SpeechAction, SpeechGoal

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

DEBUG = True

POINT = "POINT"
LABEL = "LABEL"
LASER_ON = "LASER_ON"
LASER_OFF = "LASER_OFF"

POINT_TIME = 5.0


'''
"What is this [robot points at specific object]?",
"Where is the _____?",
"Can you describe and show me an object that is similarly sized?",
"Could you place ___ next to _____?",
"Can you tell me more about ______?",
"What would you use this  [robot points at specific object] object for?",
"How are ______ and ______ similar?",
"How would you describe ________?",
"How would you use ______?",
"Can you describe one of the objects that you would use everyday?",
"How is _______ different from _________?",
"Could you sort all the objects by how useful you would find them?"
'''

def get_stamped_point(x,y,z):
	p = PointStamped()
	p.header.frame_id = "map"
	p.header.stamp = rospy.Time.now()
	p.point.x = x
	p.point.y = y
	p.point.z = z

	return p

def get_goal_pose(theta):
	goal = MoveBaseGoal()
	q = quaternion_from_euler(0.0,  0.0, theta)
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = 0.0
	goal.target_pose.pose.position.y = 0.0
	goal.target_pose.pose.position.z = 0.0

	goal.target_pose.pose.orientation.x = q[0]
	goal.target_pose.pose.orientation.y = q[1]
	goal.target_pose.pose.orientation.z = q[2] 
	goal.target_pose.pose.orientation.w = q[3]

	return goal

def distance(a, b):
	dist = math.sqrt(
		(a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2
	)

	return dist

class SoundBoard:
	def __init__(self):
		'''
		self.questions = [
			["What is this", POINT], 
			["Where is the", LABEL], 
			["Can you describe this", POINT, "and show me an object that is similarly sized"],
			["Could you place this", POINT, "next to this", POINT],
			["Can you tell me more about this", POINT],
			["What would you use this", POINT, "object for"],
			["How are this", POINT, "and that", POINT, " similar"],
			["How would you describe this", POINT],
			["How would you use this", POINT],
			["Can you describe one of the objects that you would use everyday"],
			["How is this", POINT, "different from that", POINT],
			["Could you sort all the objects by how useful you would find them"]
		]
		'''

		self.questions = [
			["How are this", POINT, "and that", POINT, " similar"],
			[POINT, "How are this", POINT, "and that similar"]
		]


		self.statements = [
			"yes",
			"no",
			"thank you",
			"how are you?",
			"here"
		]

		'''
		self.objects = []
		#TODO get rid of hard path link
		f = open("/home/iral/objects.txt", 'r')
		for line in f:
			label,x,y,z = line.split(',')
			self.objects.append( (label, float(x), float(y) ,float(z)) )
		f.close()
		'''

		rospy.init_node('experiment', anonymous = True)

		self.soundhandle = actionlib.SimpleActionClient('tts', SpeechAction)
		self.soundhandle.wait_for_server()

		#unneeded for not, used if the robot needs to turn to face something
		#self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		#self.client.wait_for_server()

		self.point_publisher = rospy.Publisher('clicked_point', PointStamped, queue_size=10)

		self.tf_listener = tf.TransformListener()

		self.sub = rospy.Subscriber("object_markers", MarkerArray, self.objects_cb)
		self.objects = []


		context = zmq.Context()
		self.socket = context.socket(zmq.PAIR)
		self.socket.connect("tcp://130.85.202.6:5555")

		#rospy.sleep(1)

	def object_cb(self, marker_array):
		new_objects = []
		for marker in marker_array:
			new_objects.append( (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, marker.header.frame_id) )
 		
		#if robot has not seen any objects
		if len(self.objects) == 0:
			self.objects = objects
		#robot has seen objects
		else:
			tmp_old_indxs = range(len(self.objects))
			tmp_new_indxs = range(len(new_objects))

			for i, new_obj in enumerate(new_objects):
				for j, old_obj in enumerate(self.objects):
					if distance(old_obj, new_obj) < DISTANCE_THRESHOLD:
						tmp_old_indxs.remove(j)
						tmp_new_indxs.remove(i)

			print(i,j)

			if len(tmp_old_indxs) > 0 and len(tmp_new_indxs) == 0:
				print("case 1")
			elif len(tmp_old_indxs) == 0 and len(tmp_new_indxs) > 0:
				print("case 2")
			elif len(tmp_old_indxs) > 0 and len(tmp_new_indxs) > 0:
				print("case 3")
			

	def high_level_prompt(self):
		print("0: Question")
		print("1: Statement")
		print("else: Quit")

		return input()

	def prompt_questions(self):
		for i,q in enumerate(self.questions):
			s = str(i)+": "+self.format_question(q)
			print(s)

		return input()

	def format_question(self, question):
		s = ""
		for q in question:
			if q == LABEL or q == POINT: 
				s += " _____ "
			elif q != LASER_ON and q != LASER_OFF:
				s += q

		return s

	def prompt_objects(self):
		for i,o in enumerate(self.objects):
			s = str(i)+": "+str(o[0])
			print(s)

		obj = input("Object:")
		while not 0 <= obj < len(self.objects):
			for i,o in enumerate(self.objects):
				s = str(i)+": "+str(o[0])
				print(s)
			obj = input("Object:")

		return obj


	def prompt_misc(self):
		for i,s in enumerate(self.statements):
			print(i,": ",s)

		s = input()

		if int(s) >= 0 and int(s) < len(self.statements):
			if DEBUG: print('DEBUG Saying: %s' % self.statements[s])
			
			goal = SpeechGoal()
			goal.text = self.statements[s]
			goal.metadata = ''

			self.soundhandle.send_goal(goal)
			self.soundhandle.wait_for_result()

	def prompt_commands(self):
		print("0: Turn laser on")
		print("1: Turn laser off")
		cmd = input()

		if cmd == 0:
			if DEBUG: print("DEBUG: LASER_ON")
			self.socket.send(LASER_ON)
		elif cmd == 1:
			if DEBUG: print("DEBUG: LASER_ON")
			self.socket.send(LASER_ON)
	
	#given a question prompt the user for all the required information needed
	def build_sentence(self, question):
		objs = []
		o = 0
		l = 0

		#Get the objects that will be referenced
		#either location or the label
		for i, w in enumerate(question):
			if w == POINT:
				obj = self.prompt_objects()
				objs.append(obj)
				need_laser = True
			elif w == LABEL:
				label = input("object name: ")
				if i > 0:
					if question[i-1] != POINT:
						question[i-1] += " " + label
						del question[i]

		#get the robot the say the question
		self.say_sentance(question, objs)

		#check if the robot needs to say it again
		repeat = raw_input("Repeat(y/n):")
		while repeat != "n":
			self.say_sentance(question, objs)
			repeat = raw_input("Repeat(y/n):")

		#give an affermation so the subject doesn't ramble on too much
		response = "OK"
		if DEBUG: print('DEBUG Saying: %s' % response)
		goal = SpeechGoal()
		goal.text = response
		goal.metadata = ''
		self.soundhandle.send_goal(goal)
		self.soundhandle.wait_for_result()

	#speak the sentance and point the laser at objects
	def say_sentance(self, question, objs):
		o = 0
		laser_on = False

		#go through the qestion
		for i in question:
			#point at objects
			if i == POINT:
				#self.face_point(objs[o][1], objs[o][2], objs[o][3])
				if DEBUG: print('DEBUG pointing at object: %d' % o)
				self.publish_point(self.objects[objs[o]][0], self.objects[objs[o]][1], self.objects[objs[o]][2])
				if DEBUG: print("DEBUG: LASER_ON")
				laser_on = True
				self.socket.send(LASER_ON)
				o += 1
			#speak
			else:
				if DEBUG: print('DEBUG Saying: %s' % i)
				goal = SpeechGoal()
				goal.text = i
				goal.metadata = ''

				self.soundhandle.send_goal(goal)
				self.soundhandle.wait_for_result()


		if laser_on:
			time.sleep(POINT_TIME)
			if DEBUG: print("DEBUG: LASER_OFF")
			self.socket.send(LASER_OFF)

	'''
	#Uneeded for now, could be used to get the robot to turn to face objects if needed
	#
	def face_point(self, x,y,z):
		#print("facing ",x,y,z)
		p = get_stamped_point(x,y,z)

		#wait for current transform to be published		
		self.tf_listener.waitForTransform("/map", "/base_link", rospy.Time.now(), rospy.Duration(5.0))
		#transform the point from whatever frame it came from to the pantil's frame
		base_pt = self.tf_listener.transformPoint("/base_link", p)

		px = base_pt.point.x
		py = base_pt.point.y

		d = math.sqrt(px*px+py*py)
		theta_cos = math.acos(px/d)

		if y<0:
			theta = -theta_cos
		else:
			theta = theta_cos

		goal = get_goal_pose(theta)

   		self.client.send_goal(goal)
    		wait = self.client.wait_for_result()
    		if not wait:
        		rospy.logerr("Action server not available!")
        		rospy.signal_shutdown("Action server not available!")
	'''

	#publish the point to get the laser pointer to point at that point	
	def publish_point(self, x,y,z):
		p = get_stamped_point(x,y,z)

		#the points read in from the object files are in the "/map" frame of reference

		#wait for current transform to be published		
		self.tf_listener.waitForTransform("/pantilt_link", "/map", rospy.Time.now(), rospy.Duration(4.0))
		#transform the point from the the "/map" frame to the pantil's frame
		pan_tilt_pt = self.tf_listener.transformPoint("/pantilt_link", p)

		self.point_publisher.publish(pan_tilt_pt)

	def run(self):
		a = 0		
		while True:
			a = self.high_level_prompt()
			if a == 0:
				
				q = self.prompt_questions()
				if q >=0 and q < len(self.questions):
					print(self.format_question(self.questions[q]))
					self.build_sentence(copy(self.questions[q]))
			elif a== 1:
				self.prompt_misc()
			elif a== 1:
				self.prompt_commands()
			else:
				break


if __name__ == '__main__':
	sb = SoundBoard()
	sb.run()


