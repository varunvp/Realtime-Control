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

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

rospy.init_node("simple_marker")

uav1_pub = rospy.Publisher("uav1_desired_pos", Point, queue_size=3)
uav2_pub = rospy.Publisher("uav2_desired_pos", Point, queue_size=3)

def processFeedback1(feedback):
    p = feedback.pose.position
    dest = Point()
    dest = feedback.pose.position
    uav1_pub.publish(dest)
    # print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

def processFeedback2(feedback):
    p = feedback.pose.position
    dest = Point()
    dest = feedback.pose.position
    uav2_pub.publish(dest)
    # print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

if __name__=="__main__":
    
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")
    
    # create an interactive marker for our server
    int_marker_1 = InteractiveMarker()
    int_marker_1.header.frame_id = "local_origin"
    int_marker_1.name = "UAV 1"
    int_marker_1.description = "Control for UAV 1"

    # create a grey box marker
    box_marker_1 = Marker()
    box_marker_1.type = Marker.CUBE
    # box_marker_1.pose.position.y = 4
    # box_marker_1.pose.position.x = 0
    box_marker_1.scale.x = 0.45
    box_marker_1.scale.y = 0.45
    box_marker_1.scale.z = 0.45
    box_marker_1.color.r = 0.0
    box_marker_1.color.g = 0.5
    box_marker_1.color.b = 0.5
    box_marker_1.color.a = 1.0

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
    int_marker_2.header.frame_id = "local_origin"
    int_marker_2.name = "UAV 2"
    int_marker_2.description = "Control for UAV 2"

    box_marker_2 = Marker()
    box_marker_2.type = Marker.CUBE
    # box_marker_2.pose.position.y = -4
    # box_marker_2.pose.position.x = 0
    box_marker_2.scale.x = 0.45
    box_marker_2.scale.y = 0.45
    box_marker_2.scale.z = 0.45
    box_marker_2.color.r = 0.5
    box_marker_2.color.g = 0.0
    box_marker_2.color.b = 0.5
    box_marker_2.color.a = 1.0
    
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

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    # rotate_control = InteractiveMarkerControl()
    # rotate_control.name = "move_x"
    # rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    # # add the control to the interactive marker
    # int_marker_1.controls.append(rotate_control);

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker_1, processFeedback1)
    server.insert(int_marker_2, processFeedback2)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()
