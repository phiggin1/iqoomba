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

if __name__ == '__main__':
    rospy.init_node('experiment', anonymous = True)
    soundhandle = SoundClient()
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    point_publisher = rospy.Publisher('clicked_point', PointStamped, queue_size=10)
    rospy.sleep(1)

    voice = 'voice_kal_diphone'
    volume = 1.0

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    p = PointStamped()
    p.header.frame_id = "pantilt_link"
    '''
    print("waiting for action server")
    client.wait_for_server()
    '''
    goal.target_pose.header.stamp = rospy.Time.now()
    yaw = math.pi/2
    goal.target_pose.pose.orientation = Quaternion(*np.asarray(quaternion_from_euler(0.0, 0.0, yaw)))
    '''
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    '''
    p.header.stamp = rospy.Time.now()
    p.point.x = 0.5
    p.point.y = 0.0
    p.point.z = 0.0
    point_publisher.publish(p)

    s = "what is this?"
    print('Saying: %s' % s)
    soundhandle.say(s, voice, volume)
    rospy.sleep(5.0)

    p.header.stamp = rospy.Time.now()
    p.point.x = 0.04
    p.point.y = 0.0
    p.point.z = -1.0
    point_publisher.publish(p)

    #=============================================================

    goal.target_pose.header.stamp = rospy.Time.now()
    yaw = 0.0
    goal.target_pose.pose.orientation = Quaternion(*np.asarray(quaternion_from_euler(0.0, 0.0, yaw)))
    '''
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    '''
    p.header.stamp = rospy.Time.now()
    p.point.x = 0.3
    p.point.y = 0.3
    p.point.z = 0.0
    point_publisher.publish(p)

    s = "what is that?"
    print('Saying: %s' % s)
    soundhandle.say(s, voice, volume)
    rospy.sleep(5.0)

    p.header.stamp = rospy.Time.now()
    p.point.x = 0.04
    p.point.y = 0.0
    p.point.z = -1.0
    point_publisher.publish(p)
