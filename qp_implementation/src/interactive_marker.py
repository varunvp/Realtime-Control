#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy, time

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from qp_planner.msg import Obstacles, CircleObstacle
from mavros_msgs.msg import HomePosition
from nav_msgs.msg import Odometry

obstacles = [[], [], [], [], []]
drone_obstacles = Obstacles()
drone_obstacles.circles = [CircleObstacle(), CircleObstacle(), CircleObstacle(), CircleObstacle()]

rospy.init_node("simple_marker")

uav1_pub = rospy.Publisher("uav0/desired_pos", Point, queue_size=3)
uav2_pub = rospy.Publisher("uav1/desired_pos", Point, queue_size=3)
uav3_pub = rospy.Publisher("uav2/desired_pos", Point, queue_size=3)
uav4_pub = rospy.Publisher("uav3/desired_pos", Point, queue_size=3)

publish_drones_pos = rospy.Publisher('drones_state', Obstacles, queue_size = 10)

def processFeedback1(feedback):
    dest = Point()
    dest = feedback.pose.position
    uav1_pub.publish(dest)

def processFeedback2(feedback):
    dest = Point()
    dest = feedback.pose.position
    uav2_pub.publish(dest)

def processFeedback3(feedback):
    dest = Point()
    dest = feedback.pose.position
    uav3_pub.publish(dest)

def processFeedback4(feedback):
    dest = Point()
    dest = feedback.pose.position
    uav4_pub.publish(dest)

def receive_position_cb(data):
    global obstacles, drone_obstacles

    index = int(data.child_frame_id[3])

    drone_obstacles.circles[index].center.x = data.pose.pose.position.x
    drone_obstacles.circles[index].center.y = data.pose.pose.position.y
    drone_obstacles.circles[index].radius = 1
    drone_obstacles.circles[index].velocity.x = data.twist.twist.linear.x
    drone_obstacles.circles[index].velocity.y = data.twist.twist.linear.y

    publish_drones_pos.publish(drone_obstacles)

if __name__=="__main__":
    # rospy.init_node('drone_stat_publisher')
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")
    # create an interactive marker for our server
    int_marker_1 = InteractiveMarker()
    int_marker_1.header.frame_id = "map"
    int_marker_1.name = "UAV 1"
    int_marker_1.description = "Control for UAV 1"

    # create a grey box marker
    box_marker_1 = Marker()
    box_marker_1.type = Marker.CUBE
    int_marker_1.pose.position.y = -2
    int_marker_1.pose.position.x = -2
    box_marker_1.scale.x = 0.45
    box_marker_1.scale.y = 0.45
    box_marker_1.scale.z = 0.45
    box_marker_1.color.r = 1.0
    box_marker_1.color.g = 0.0
    box_marker_1.color.b = 0.0
    box_marker_1.color.a = 0.2

    # create a non-interactive control which contains the box
    uav_control_1 = InteractiveMarkerControl()
    uav_control_1.always_visible = True
    uav_control_1.orientation.w = 1
    uav_control_1.orientation.x = 0
    uav_control_1.orientation.y = 1
    uav_control_1.orientation.z = 0
    uav_control_1.name = "move_uav1"
    uav_control_1.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker_1.controls.append( uav_control_1 )
    uav_control_1.markers.append( box_marker_1 )

    int_marker_2 = InteractiveMarker()
    int_marker_2.header.frame_id = "map"
    int_marker_2.name = "UAV 2"
    int_marker_2.description = "Control for UAV 2"

    box_marker_2 = Marker()
    box_marker_2.type = Marker.CUBE
    int_marker_2.pose.position.y = -2
    int_marker_2.pose.position.x = 2
    box_marker_2.scale.x = 0.45
    box_marker_2.scale.y = 0.45
    box_marker_2.scale.z = 0.45
    box_marker_2.color.r = 0.0
    box_marker_2.color.g = 1.0
    box_marker_2.color.b = 0.0
    box_marker_2.color.a = 0.2
    
    uav_control_2 = InteractiveMarkerControl()
    uav_control_2.always_visible = True
    uav_control_2.orientation.w = 1
    uav_control_2.orientation.x = 0
    uav_control_2.orientation.y = 1
    uav_control_2.orientation.z = 0
    uav_control_2.name = "move_uav2"
    uav_control_2.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker_2.controls.append( uav_control_2 )
    uav_control_2.markers.append( box_marker_2 )

    int_marker_3 = InteractiveMarker()
    int_marker_3.header.frame_id = "map"
    int_marker_3.name = "UAV 3"
    int_marker_3.description = "Control for UAV 3"

    box_marker_3 = Marker()
    box_marker_3.type = Marker.CUBE
    int_marker_3.pose.position.y = 2
    int_marker_3.pose.position.x = -2
    box_marker_3.scale.x = 0.45
    box_marker_3.scale.y = 0.45
    box_marker_3.scale.z = 0.45
    box_marker_3.color.r = 0.0
    box_marker_3.color.g = 0.0
    box_marker_3.color.b = 1.0
    box_marker_3.color.a = 0.2
    
    uav_control_3 = InteractiveMarkerControl()
    uav_control_3.always_visible = True
    uav_control_3.orientation.w = 1
    uav_control_3.orientation.x = 0
    uav_control_3.orientation.y = 1
    uav_control_3.orientation.z = 0
    uav_control_3.name = "move_uav3"
    uav_control_3.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker_3.controls.append( uav_control_3 )
    uav_control_3.markers.append( box_marker_3 )

    int_marker_4 = InteractiveMarker()
    int_marker_4.header.frame_id = "map"
    int_marker_4.name = "UAV 4"
    int_marker_4.description = "Control for UAV 4"

    box_marker_4 = Marker()
    box_marker_4.type = Marker.CUBE
    int_marker_4.pose.position.y = 2
    int_marker_4.pose.position.x = 2
    box_marker_4.scale.x = 0.45
    box_marker_4.scale.y = 0.45
    box_marker_4.scale.z = 0.45
    box_marker_4.color.r = 1.0
    box_marker_4.color.g = 0.0
    box_marker_4.color.b = 1.0
    box_marker_4.color.a = 0.2
    
    uav_control_4 = InteractiveMarkerControl()
    uav_control_4.always_visible = True
    uav_control_4.orientation.w = 1
    uav_control_4.orientation.x = 0
    uav_control_4.orientation.y = 1
    uav_control_4.orientation.z = 0
    uav_control_4.name = "move_uav4"
    uav_control_4.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker_4.controls.append( uav_control_4 )
    uav_control_4.markers.append( box_marker_4 )

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker_1, processFeedback1)
    server.insert(int_marker_2, processFeedback2)
    server.insert(int_marker_3, processFeedback3)
    server.insert(int_marker_4, processFeedback4)

    # 'commit' changes and send to all clients
    server.applyChanges()

    time.sleep(1)
        
    home_latlong = HomePosition()
    home_latlong.geo.latitude = 13.027207
    home_latlong.geo.longitude = 77.563642
    home_latlong.geo.altitude = 928

    for i in range(4):
        rospy.Subscriber('/uav'+str(i)+'/mavros/global_position/local', Odometry, receive_position_cb)

        home_publisher = rospy.Publisher('/uav'+str(i)+'/mavros/global_position/home', HomePosition, queue_size = 10)

        time.sleep(0.5)
        for _ in range(0, 10):
            home_publisher.publish(home_latlong)


    rospy.spin()
