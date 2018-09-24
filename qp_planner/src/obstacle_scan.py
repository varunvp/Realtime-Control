#!/usr/bin/python

from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from sklearn.cluster import KMeans

import numpy as np

import rospy

def get_points(data):
	points = np.array(data.ranges)


# rospy.init_node('Obstacle_Cluster')

# if __name__ == "__main__":

