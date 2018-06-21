#!/usr/bin/python

from __future__ import print_function

import os
import sys
import select
import termios
import tty
import roslib
import rospy
import argparse
import rospy
import mavros
import threading
import time
import readline
import signal
import select
import tf

from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from subprocess import call
from mavros_msgs.msg import OverrideRCIn
from mavros import command
from mavros import setpoint as SP
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker

from qp_planner.msg import algomsg
from mavros_msgs.msg import Altitude
from gazebo_msgs.msg import ModelStates

from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion
import quadprog
import numpy
from numpy import array
import math
from qp_matrix import qp_q_dot_des_array

global R
global roll, pitch, yaw


cont = 0
home_xy_recorded = home_z_recorded = False
cart_x = cart_y = cart_z = 0.0
home_x = home_y = home_z = 0.0
ekf_x = ekf_y = ekf_z = 0
desired_x = desired_y =  desired_z = 0.0
limit_x = limit_y = limit_z = 5.0
roll = pitch = yaw = 0.0
TIMEOUT = 0.5
kp = 1.
kb = 10000000.0
home_yaw = 0
br = tf.TransformBroadcaster()
br2 = tf.TransformBroadcaster()
y_pub = rospy.Publisher('y_graph', Float32, queue_size = 5)
discard_samples = 20                        #samples to discard before gps normalizes
pos = Point()
quat = Quaternion()
pos.x = pos.y = pos.z = 0
quat.x = quat.y = quat.z = quat.w = 0
start_y = 0.0


def quadprog_solve_qp(H, h, A=None, b=None, C=None, d=None):
    qp_H = .5 * (H + H.T)  # make sure H is symmetric
    qp_h = -h
    if C is not None:
        qp_C = -numpy.vstack([C, A]).T
        qp_d = -numpy.hstack([d, b])
        meq = C.shape[0]
    else:  # no equality constraint
        qp_C = -A.T
        qp_d = -b
        meq = 0

    # print qp_H
    # print qp_h
    # print qp_C
    # print qp_d
    return quadprog.solve_qp(qp_H, qp_h, qp_C, qp_d, meq)[0]


def qp_q_dot_des(q_act, q_des, q_origin, q_limit, q_kp, q_kb):
    # q_des = 0.

    # cost function matrix is given here   e.g. u^T H u
    H = array([[1000., 0.], [0., 1.]])
    h = array([0., 0.])  # cost function vector    e.g. h^T u

    # stability constraints
    kp = q_kp
    Va = q_act - q_des
    Vb = -kp * (q_act - q_des) * (q_act - q_des)

    # # safety constraints
    limit = q_limit  # in kms for position and radians for angles - very high
    q_rel = q_act - q_origin
    Ba = - 2. * q_rel  # derivative of angle_limit - x^2
    Bb = -q_kb * (limit * limit - q_rel * q_rel)  # - (angle_limit - x^2)

    # inequality constraints are given here Au \leq b
    A = array([[-1, Va], [0, -Ba]])
    b = array([Vb, -Bb])
    # A = array([[-1, Va]])
    # b = array([Vb])
    
    u_in = quadprog_solve_qp(H, h, A, b)

    return array([u_in[1]])


def imu_cb(data):
    global roll, pitch, yaw
    # imu_data.x = data.linear_acceleration.x
    # imu_data.y = data.linear_acceleration.y
    # imu_data.z = data.linear_acceleration.z
    orientation_q = data.orientation
    orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    (roll, pitch, yaw) = (roll * 180.0/3.1416, pitch * 180.0/3.1416, yaw  * 180.0/3.1416)


def gps_global_cb(data):
    R = 6371000
    lat = data.latitude
    lon = data.longitude
    alt = data.altitude

    global cart_x, cart_y, cart_z, home_x, home_y, home_xy_recorded, discard_samples

    cart_x = R * math.cos(lat) * math.cos(lon)
    cart_y = R * math.cos(lat) * math.sin(lon)
    # cart_z = R * math.sin(lat)

    if home_xy_recorded is False and cart_x != 0 and cart_y != 0:
        home_x = cart_x
        home_y = cart_y

        discard_samples = discard_samples - 1

        if(discard_samples <= 0):
            home_xy_recorded = True


def gps_local_cb(data):
    global cart_x, cart_y, home_x, home_y, home_xy_recorded, discard_samples, desired_x, desired_y, start_y

    cart_x = data.pose.pose.position.x
    cart_y = data.pose.pose.position.y
    # cart_z = data.pose.pose.position.z

    if home_xy_recorded is False and cart_x != 0 and cart_y != 0:
        home_x = cart_x
        home_y = cart_y
        discard_samples = discard_samples - 1

        if(discard_samples <= 0):
            desired_x = cart_x                          #to set home position as initial desired position
            desired_y = cart_y
            start_y = home_y
            home_xy_recorded = True


def pose_cb(data):
    global ekf_x, ekf_y,ekf_z
    # orientation_q = data.pose.orientation
    # orientation_list = (orientation_q.x, orientation_q.y,
    #                     orientation_q.z, orientation_q.w)
    # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # (roll, pitch, yaw) = (roll * 180.0/3.1416, pitch * 180.0/3.1416, yaw  * 180.0/3.1416)
    position = data.pose.position
    ekf_x = position.x
    ekf_y = position.y
    ekf_z = position.z


def calc_target_cb(data):
    global desired_x, desired_y, desired_z
    desired_x = home_x + data.pose.position.x / 1
    desired_y = home_y + data.pose.position.y / 1
    # desired_z = home_z + data.pose.position.z


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


def algomsg1(algo, command, speed, abs_speed, precedence, x, y, z, a, b, c):
    cmd = algomsg(header=Header(stamp=rospy.get_rostime()))
    cmd.algo = algo
    cmd.command = command
    cmd.speed = speed
    cmd.abs_speed = abs_speed
    cmd.precedence = precedence
    cmd.cmd_custom = twist_obj(x, y, z, a, b, c)
    return cmd


def alt_cb(data):
    global home_z, cart_z, home_z_recorded
    cart_z = data.monotonic / 1


def compact_algomsg(abs_speed, x, y, z, yaw):
    algomsg1('Road-Following', 'custom', 0, abs_speed, 1, x, y, z, 0, 0, yaw)


def gazebo_cb(data):
    global br
    global cont,rate, pos, quat 

    pos = data.pose[1].position
    quat = data.pose[1].orientation


def ekf_cb(data):
    data.pose.position.x 


def plot(vel_y):
    global cart_y, desired_y, start_y

    timer = 0

    if(math.fabs(vel_y) >= 0.01):
        y_diff = Float32()
        try:
            y = math.fabs((cart_y - start_y))/math.fabs((desired_y - start_y))
            if(y < 2):
                y_diff.data = y
                y_pub.publish(y_diff)

        except ZeroDivisionError as e:
            print("Oops")

        # print(cart_y - start_y,desired_y - start_y, start_y,y_diff.data)

    elif(math.fabs(vel_y) < 0.01):
        start_y = desired_y

        print("-------------------Ready-------------------")


def main():
    global home_xy_recorded, home_z_recorded, cart_x, cart_y, cart_z, desired_x, desired_y, desired_z, home_yaw
    global home_x, home_z, home_y, limit_x, limit_y, limit_z, kp, kb, roll, pitch, yaw, cont
    xAnt = yAnt = 0
    home_xy_recorded = False
    rospy.init_node('MAVROS_Listener')
    
    rate = rospy.Rate(100.0)

    rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
    # rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_global_cb)
    rospy.Subscriber("/mavros/global_position/local", Odometry, gps_local_cb)
    rospy.Subscriber("/mavros/altitude", Altitude, alt_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, ekf_cb)

    pub = rospy.Publisher('destination_point', PointStamped, queue_size = 1)
    pub2 = rospy.Publisher('gps_point', PointStamped, queue_size = 5)
    pub3 = rospy.Publisher('boundary_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher('path', Path, queue_size=1)
    pub5 = rospy.Publisher('ekf_path', Path, queue_size=1)

    path = Path()
    ekf_path = Path()
    max_append = 1000

    # rospy.spin()
    while not rospy.is_shutdown():
    	yaw = 360.0 + yaw if yaw < 0 else yaw

        if home_z_recorded is False and cart_z != 0 and yaw != 0:
            desired_z = cart_z + 3
            home_yaw = yaw
            home_z = cart_z
            home_z_recorded = True

        ready = select.select([sys.stdin], [], [], 0)[0]

        if ready:
            x = ready[0].read(1)

            if x == 'x':
                limit_x = float(raw_input('Enter x limit:'))

            if x == 'y':
                limit_y = float(raw_input('Enter y limit:'))

            if x == 'z':
                limit_z = float(raw_input('Enter z limit:'))

            if x == 'k':
                kp = float(raw_input('Enter kp limit:'))

            if x == 'b':
                kb = float(raw_input('Enter kb limit:'))

            sys.stdin.flush()

        # +90 To account for diff between world and drone frame
        # desired_yaw = (math.atan2(desired_y - cart_y, desired_x - cart_x) * 180 / 3.1416)
        # desired_yaw = 360.0 + desired_yaw if desired_yaw < 0 else desired_yaw

        cart_array = [cart_x, cart_y, cart_z]
        des_array = [desired_x, desired_y, desired_z]
        origin_array = [home_x, home_y, home_z]
        limit_array = [limit_x, limit_y, limit_z]
        kp_array = [kp, kp, kp]
        kb_array = [kb, kb, kb]
        v_des = qp_q_dot_des_array(cart_array, des_array, origin_array, limit_array, kp_array, kb_array)
        velocity_x_des = v_des[0]
        velocity_y_des = v_des[1]
        velocity_z_des = v_des[2]
        # velocity_x_des = qp_q_dot_des(cart_x, desired_x, home_x, limit_x, kp, kb)
        # velocity_y_des = qp_q_dot_des(cart_y, desired_y, home_y, limit_y, kp, kb)
        # velocity_z_des = qp_q_dot_des(cart_z, desired_z, home_z, limit_z, kp, kb)
        # velocity_yaw_des = -qp_q_dot_des(yaw, desired_yaw, home_yaw, 36000, 1000, 1000)
        # print(velocity_y_des)
        # velocity_x_des = 0
        # velocity_y_des = 0
        # velocity_z_des = -10 * (cart_z - desired_z)
        # velocity_yaw_des = 0

        # pub = rospy.Publisher('/algo/roadfollowing', algomsg, queue_size=10)
        # pub.publish(algomsg1("Road-Following", 'custom', 'medium', 0, 1, velocity_x_des, velocity_y_des, velocity_z_des, 0.0, 0.0, 0.0))

        velocity_x_des = velocity_x_des if math.fabs(velocity_x_des) < 2.5 else 2.5 * math.copysign(1, velocity_x_des)
        velocity_y_des = velocity_y_des if math.fabs(velocity_y_des) < 2.5 else 2.5 * math.copysign(1, velocity_y_des)

        pub1.publish(twist_obj(velocity_x_des, velocity_y_des, velocity_z_des, 0.0, 0.0, 0.0))
        print (cart_x, cart_y, cart_z, home_x, home_y, home_z, velocity_x_des, velocity_y_des, velocity_z_des)
        # print((cart_x - home_x), (cart_y - home_y), desired_x - home_x, desired_y - home_y)
        # print(desired_x, desired_y)

        desired_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        desired_point.header.frame_id = 'local_origin'
        desired_point.point.x = desired_x - home_x
        desired_point.point.y = desired_y - home_y
        desired_point.point.z = 0
        pub.publish(desired_point)

        gps_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        gps_point.header.frame_id = 'local_origin'
        gps_point.point.x = (cart_x - home_x)
        gps_point.point.y = (cart_y - home_y)
        gps_point.point.z = (cart_z - home_z)
        pub2.publish(gps_point)

        ekf_pose = PoseStamped()
        ekf_pose.header.frame_id = "local_origin"
        ekf_pose.pose.position.x = ekf_x
        ekf_pose.pose.position.y = ekf_y
        ekf_pose.pose.position.z = ekf_z

        boundary_cube = Marker()
        boundary_cube.header.frame_id = 'local_origin'
        boundary_cube.header.stamp = rospy.Time.now()
        boundary_cube.action = boundary_cube.ADD
        boundary_cube.type = boundary_cube.CUBE
        boundary_cube.id = 0
        boundary_cube.scale.x = limit_x*2
        boundary_cube.scale.y = limit_y*2
        boundary_cube.scale.z = limit_z*2       
        boundary_cube.color.a = 0.5 
        boundary_cube.color.r = 0.4
        boundary_cube.color.g = 0.2
        boundary_cube.color.b = 0
        pub3.publish(boundary_cube)

        # plot(velocity_y_des)

        pose = PoseStamped()
        pose.header.frame_id = "local_origin"
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z
        pose.pose.orientation.x = quat.x
        pose.pose.orientation.y = quat.y
        pose.pose.orientation.z = quat.z
        pose.pose.orientation.w = quat.w

        # if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
        pose.header.seq = path.header.seq + 1
        path.header.frame_id = "local_origin"
        path.header.stamp = rospy.Time.now()
        pose.header.stamp = path.header.stamp
        path.poses.append(pose)
        ekf_pose.header.seq = ekf_path.header.seq + 1
        ekf_path.header.frame_id = "local_origin"
        ekf_path.header.stamp = rospy.Time.now()
        ekf_pose.header.stamp = ekf_path.header.stamp
        ekf_path.poses.append(ekf_pose)
        cont = cont + 1

        xAnt = pose.pose.position.x
        yAnt = pose.pose.position.y

        pub4.publish(path)
        pub5.publish(ekf_path)
        # print(pose.pose.position.x, pose.pose.position.y)

        if cont > max_append and len(path.poses) != 0 and len(ekf_path.poses):
                path.poses.pop(0)
                ekf_path.poses.pop(0)

        br.sendTransform((pos.x, pos.y, pos.z), (quat.x, quat.y, quat.z, quat.w), rospy.Time.now(), "base_link", "local_origin")
        br2.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "fcu", "local_origin")
       


if __name__ == "__main__":
    mavros.set_namespace("/mavros")
    pub1 = SP.get_pub_velocity_cmd_vel(queue_size=3)
    path = Path() 

    main()
