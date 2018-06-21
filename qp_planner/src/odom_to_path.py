#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy
import sys
import json
from collections import deque
from gazebo_msgs.msg import ModelStates

import time
rate = 0

def callback(data):
        global xAnt
        global yAnt
        global cont,rate

        pose = PoseStamped()

        pose.header.frame_id = "local_origin"
        pose.pose.position.x = float(data.pose[1].position.x)
        print(data.pose[1].position.x)
        pose.pose.position.y = float(data.pose[1].position.y)
        pose.pose.position.z = float(data.pose[1].position.z)
        pose.pose.orientation.x = float(data.pose[1].orientation.x)
        pose.pose.orientation.y = float(data.pose[1].orientation.y)
        pose.pose.orientation.z = float(data.pose[1].orientation.z)
        pose.pose.orientation.w = float(data.pose[1].orientation.w)

        if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
                pose.header.seq = path.header.seq + 1
                path.header.frame_id = "local_origin"
                path.header.stamp = rospy.Time.now()
                pose.header.stamp = path.header.stamp
                path.poses.append(pose)
                # Published the msg

        cont = cont + 1

        # rospy.loginfo("Hit: %i" % cont)
        if cont > max_append:
                path.poses.pop(0)

        print(pose.pose.position.x, pose.pose.position.y)
        pub.publish(path)

        xAnt = pose.pose.orientation.x
        yAnt = pose.pose.position.y

        return path


if __name__ == '__main__':
        # Initializing global variables
        global xAnt
        global yAnt
        global cont, rate
        xAnt = 0.0
        yAnt = 0.0
        cont = 0

        # Initializing node
        rospy.init_node('path_plotter')

        rate = rospy.Rate(1000)

        # Rosparams set in the launch (can ignore if running directly from bag)
        # max size of array pose msg from the path
        if not rospy.has_param("~max_list_append"):
                rospy.logwarn('The parameter max_list_append dont exists')
        max_append = rospy.set_param("~max_list_append", 1000)
        max_append = 1000
        if not (max_append > 0):
                rospy.logwarn('The parameter max_list_append is not correct')
                sys.exit()
        pub = rospy.Publisher('/path', Path, queue_size=10)

        path = Path() 
        # msg = ModelStates()

        # Subscription to the required odom topic (edit accordingly)
        rospy.Subscriber("/gazebo/model_states", ModelStates, callback)


        try:
                while not rospy.is_shutdown():
                    # rospy.spin()
                    rate.sleep()
        except rospy.ROSInterruptException:
                pass
