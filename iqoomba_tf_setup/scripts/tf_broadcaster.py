#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
        rospy.init_node('static_tf2_kinect_pantilt')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
 
        #transform from the kinect_link to the pantilt_link
        kinect_to_pantilt = geometry_msgs.msg.TransformStamped()
        kinect_to_pantilt.header.stamp = rospy.Time.now()
        kinect_to_pantilt.header.frame_id = "kinect2_link"
        kinect_to_pantilt.child_frame_id = "pantilt_link"
        '''
        kinect_to_pantilt.transform.translation.x = -0.03
        kinect_to_pantilt.transform.translation.y = -0.10
        kinect_to_pantilt.transform.translation.z = 0.07
        '''

        kinect_to_pantilt.transform.translation.x = -0.037
        kinect_to_pantilt.transform.translation.y = -0.065
        kinect_to_pantilt.transform.translation.z = 0.07

        kinect_to_pantilt.transform.rotation.x = 0.5
        kinect_to_pantilt.transform.rotation.y = -0.5
        kinect_to_pantilt.transform.rotation.z = 0.5
        kinect_to_pantilt.transform.rotation.w = 0.5

        #transform from the create2 base_link to the kinect_link
        base_to_kinect = geometry_msgs.msg.TransformStamped()
        base_to_kinect.header.stamp = rospy.Time.now()

        base_to_kinect.header.frame_id = "base_link"
        base_to_kinect.child_frame_id = "kinect2_link"

        base_to_kinect.transform.translation.x = 0.14
        base_to_kinect.transform.translation.y = -0.08
        base_to_kinect.transform.translation.z = 0.09

        base_to_kinect.transform.rotation.x = -0.5
        base_to_kinect.transform.rotation.y = 0.5
        base_to_kinect.transform.rotation.z = -0.5
        base_to_kinect.transform.rotation.w = 0.5

        broadcaster.sendTransform([kinect_to_pantilt, base_to_kinect])

        rospy.spin()
