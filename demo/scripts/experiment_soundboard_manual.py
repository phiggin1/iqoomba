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
import random
from copy import copy
from sound_play.libsoundplay import SoundClient
from tts.msg import SpeechAction, SpeechGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import MarkerArray
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

class SoundBoard:
    def __init__(self):

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
        '''


        self.statements = [
            "yes",
            "thank you"
        ]

        self.objects = dict()

        rospy.init_node('experiment', anonymous = True)

        self.soundhandle = actionlib.SimpleActionClient('tts', SpeechAction)
        self.soundhandle.wait_for_server()

        self.point_publisher = rospy.Publisher('clicked_point', PointStamped, queue_size=10)

        tracked_objects = rospy.wait_for_message('tracked_objects', MarkerArray)
        self.init_objects(tracked_objects)

        self.object_sub = rospy.Subscriber('tracked_objects', MarkerArray, self.objects_cb)

        self.tf_listener = tf.TransformListener()

        context = zmq.Context()
        self.socket = context.socket(zmq.PAIR)
        self.socket.connect("tcp://130.85.202.6:5555")

        self.log = open(str(rospy.Time.now())+".log" , "w")

    def init_objects(self, marker_array):
        for marker in marker_array.markers:
            print(marker.text)
            label = raw_input("Label: ")
            self.objects[marker.text] = (label, PointStamped())
            self.objects[marker.text][1].header = marker.header
            self.objects[marker.text][1].point = marker.pose.position

    def objects_cb(self, marker_array):
        for marker in marker_array.markers:
            if marker.text in self.objects:
                self.objects[marker.text][1].header = marker.header
                self.objects[marker.text][1].point = marker.pose.position
            else:
                label = marker.text
                self.objects[marker.text] = (label, PointStamped())
                self.objects[marker.text][1].header = marker.header
                self.objects[marker.text][1].point = marker.pose.position

    def stop(self):
        self.log.close()
        self.socket.send(LASER_OFF)

    def high_level_prompt(self):
        print("0: Question")
        print("1: Statement")
        print("2: Commands")
        print("anything else to quit")

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
        while True:
            keys = self.objects.keys()
            for i, o in enumerate(keys):
                s = str(i) + ": " + self.objects[o][0]
                print(s)
			
            response = input("Object:")
            obj = self.objects[keys[response]]

            if 0<= response < len(self.objects):
                break

        return obj


    def prompt_misc(self):
        s = raw_input("String to say:")
        if DEBUG: print('DEBUG Saying: %s' % s)
        self.say(s)    

    def prompt_commands(self):
        print("0: Turn laser on")
        print("1: Turn laser off")
        cmd = input()

        if cmd == 0:
            if DEBUG: print("DEBUG: LASER_ON")
            self.socket.send(LASER_ON)
        elif cmd == 1:
            if DEBUG: print("DEBUG: LASER_OFF")
            self.socket.send(LASER_OFF)
    
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
                obj = self.prompt_objects()
                if i > 0:
                    if question[i-1] != POINT:
                        question[i-1] += " " + self.objects[obj][0]
                        del question[i]

        #get the robot the say the question
        self.say_sentance(question, objs)

        #check if the robot needs to say it again
        repeat = raw_input("Repeat(y/n):")
        while repeat != "n":
            self.say_sentance(question, objs)
            repeat = raw_input("Repeat(y/n):")

        #give an affermation so the subject doesn't ramble on too much
        n = random.randint(0, len(self.statements)-1)
        response = self.statements[n]
        if DEBUG: print('DEBUG Saying: %s' % response)
        self.say(response)

    #speak the sentance and point the laser at objects
    def say_sentance(self, question, objs):
        o = 0
        laser_on = False

        #go through the qestion
        for i in question:
            #point at objects
            if i == POINT:
                if DEBUG: print('DEBUG pointing at: %s, %s' % (objs[o][0], objs[o][1].point))
                self.publish_point(objs[o][1])
                if DEBUG: print("DEBUG: LASER_ON")
                laser_on = True
                self.socket.send(LASER_ON)
                o += 1
            #speak
            else:
                if DEBUG: print('DEBUG Saying: %s' % i)
                self.say(i)
        if laser_on:
            time.sleep(POINT_TIME)
            if DEBUG: print("DEBUG: LASER_OFF")
            self.socket.send(LASER_OFF)

    def say(self, s):
        goal = SpeechGoal()
        goal.text = s
        goal.metadata = ''
        self.soundhandle.send_goal(goal)
        self.soundhandle.wait_for_result()

        self.log.write(str(rospy.Time.now()) + ":" + s+"\n")

    #publish the point to get the laser pointer to point at that point    
    def publish_point(self, p):
        #wait for current transform to be published        
        self.tf_listener.waitForTransform("/pantilt_link", "/map", rospy.Time.now(), rospy.Duration(4.0))
        #transform the point from the the "/map" frame to the pantil's frame
        pan_tilt_pt = self.tf_listener.transformPoint("/pantilt_link", p)

        self.point_publisher.publish(pan_tilt_pt)

    def run(self):
        #give an affermation so the subject doesn't ramble on too much
        response = "Can you tell me about the objects on the table."
        if DEBUG: print('DEBUG Saying: %s' % response)
        self.say(response)

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
            elif a== 2:
                self.prompt_commands()
            else:
                self.stop()
                break



if __name__ == '__main__':
    sb = SoundBoard()
    sb.run()
