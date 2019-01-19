#!/usr/bin/python

import numpy as np 
import quadprog, math, qp_matrix, rospy, mavros
from gazebo_msgs.msg import ModelStates
from MPC_obstacle_avoidance_challenge1 import MPC_solver
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from mavros import setpoint as SP

global K1
pos = Point()
V = np.array([[42.4146,0],
[41.4204,5.22993],
[38.4651,9.87094],
[33.6805,13.3276],
[27.2258,14.9585],
[19.0979,13.8661],
[7.78605,7.30599],
[0,0],
[0,0],
[0,0],
[0,0],
[0,0],
[0,0],
[-0,-0],
[-0,-0],
[-0,-0],
[-0,-0],
[-0,-0],
[-0,-0],
[-7.56705,-7.12318],
[-18.9728,-13.8215],
[-27.1314,-14.9629],
[-33.6079,-13.3608],
[-38.4128,-9.92271],
[-41.4001,-5.29434],
[-42.4146,-0.067517],
[-41.4405,5.16547],
[-38.5174,9.81903],
[-33.753,13.2942],
[-27.3201,14.9537],
[-19.2056,13.8977],
[-8.00023,7.48306],
[-0,0],
[-0,0],
[-0,0],
[-0,0],
[-0,0],
[-0,0],
[0,-0],
[0,-0],
[0,-0],
[0,-0],
[0,-0],
[0,-0],
[7.34281,-6.9342],
[18.8646,-13.7887],
[27.0368,-14.967],
[33.5351,-13.3938],
[38.3725,-9.97759],
[41.3679,-5.35727],
[42.4144,-0.135155]])

def twist_obj(x, y, z, a, b, c):
    # move_cmd = Twist()
    move_cmd = TwistStamped(header=Header(stamp=rospy.get_rostime()))
    move_cmd.twist.linear.x = x
    move_cmd.twist.linear.y = y
    move_cmd.twist.linear.z = z
    move_cmd.twist.angular.x = a
    move_cmd.twist.angular.y = b
    move_cmd.twist.angular.z = c
    return move_cmd

def gazebo_cb(data):
    global pos, quat 

    pos = data.pose[1].position
    quat = data.pose[1].orientation


def find_closest_point(x0, y0, points):
	min_dist = 100000,
	min_index = 0
	dist = [0] * 50
	for i in range(0, 50):
		dist[i] = math.sqrt((x0 - points[i][0])**2 + (y0 - points[i][1])**2)

		if (dist[i] < min_dist):
			min_dist = dist[i]
			min_index = i

	return min_dist, min_index


def calc_p_hat(min_index, p):
	#K1 is length of path which we generate 
	#K2 is no. of points  in ellipse
	global K1
	K1 = 20
	K2 = len(V)
	print K2
	j = min_index
	p_hat=[0] * K1
	for i in range(0, K1):
		if(min_index + i +1 <= K2):
			p_hat[i] = p[j + i + 1]

		else:
			p_hat[i] = p[j + i + 1 - K2]

	return p_hat

if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)

	global pos, K1
	x0 = pos.x
	y0 = pos.y
	min_dist, min_index = find_closest_point(x0, y0, V)
	p_hat = calc_p_hat(min_index, V)

	mavros.set_namespace("/mavros")

	rospy.init_node("controller")
	rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_cb)

	pub1 = SP.get_pub_velocity_cmd_vel(queue_size=3)

	while not rospy.is_shutdown():
		x_velocity_des, y_velocity_des, cached_var = MPC_solver([0, 0, 0], [x0, y0, 0], [0, 0, 0], x_limit=10000, y_limit=10000, nsteps=K1, interval=0.1)
		pub1.publish(twist_obj(x_velocity_des, y_velocity_des, z_velocity_des, 0.0, 0.0, 0.0))
