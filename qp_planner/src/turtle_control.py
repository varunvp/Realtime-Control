#!/usr/bin/python
import os
import rospy
import roslib
import select
import sys
import tf

from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
import quadprog
import numpy as np
from numpy import array
import math
from qp_matrix import qp_q_dot_des_array
from MPC import MPC_solver
import qp_matrix
from tf.transformations import euler_from_quaternion

current_x = current_y = current_yaw = 0.0
destination_x = destination_y = destination_yaw = 0.0
home_y =  home_yaw = 0
home_x = 0
limit_x = limit_y = 10
yaw = 0
home_yaw_recorded = False
n = 15
t = 1
br = tf.TransformBroadcaster()
cached_var = {}
lin_vel_lim = .1
ang_vel_lim = .5

def odom_cb(data):
    global current_yaw, current_x, current_y

    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    orientation_q = data.pose.pose.orientation
    orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    (roll, pitch, current_yaw) = euler_from_quaternion(orientation_list)
    (roll, pitch, current_yaw) = (0, 0, current_yaw  * 180.0/3.1416)


def calc_target_cb(data):
    global destination_x, destination_y, desired_z
    destination_x = home_x + data.pose.position.x
    destination_y = home_y + data.pose.position.y


def main():
    global home_yaw, home_yaw_recorded, home_x, home_y, current_x, current_y, current_yaw, limit_x, limit_y, n, t, cached_var, ang_vel_lim, lin_vel_lim
    global limit_x, limit_y
    xAnt = yAnt = 0

    rospy.init_node('MAVROS_Listener')
    
    rate = rospy.Rate(10.0)

    rospy.Subscriber("/odom", Odometry, odom_cb)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)

    pub = rospy.Publisher('destination_point', PointStamped, queue_size = 1)
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
    pub3 = rospy.Publisher('boundary_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher('mpc_path', Path, queue_size=1)

    mpc_path = Path()

    while not rospy.is_shutdown():
        if home_yaw_recorded is False and current_yaw != 0:
            home_yaw = current_yaw
            home_yaw_recorded = True

        ready = select.select([sys.stdin], [], [], 0)[0]

        if ready:
            x = ready[0].read(1)

            if x == 'x':
                limit_x = float(raw_input('Enter x limit:'))

            if x == 'y':
                limit_y = float(raw_input('Enter y limit:'))

            if x == 'n':
                n = int(raw_input("Enter nsteps:"))

            if x == 't':
                t = float(raw_input("Enter timestep duration:"))

            sys.stdin.flush()
       
        current_r = math.sqrt(current_x * current_x + current_y * current_y)
        destination_r = math.sqrt(math.pow(destination_x - current_x, 2) + math.pow(destination_y -current_y, 2))
        limit_r = math.sqrt(limit_x * limit_x + limit_y * limit_y)

        velocity_r_des, cached_var = MPC_solver(current_r * 0, destination_r, limit_r, home_x, n, t, True, variables = cached_var, vel_limit = lin_vel_lim)
        r_arr = cached_var.get("points")
        # velocity_y_des, cached_var = MPC_solver(current_y, destination_y, limit_y, home_y, n, t, True, variables = cached_var, vel_limit = lin_vel_lim)

        destination_yaw  = math.atan2(destination_y - current_y, destination_x - current_x) * 180 / 3.1416
        # destination_yaw = math.atan(x)
        current_yaw = 360.0 + current_yaw if current_yaw < 0 else current_yaw
        destination_yaw = 360.0 + destination_yaw if destination_yaw < 0 else destination_yaw

        if(math.fabs(destination_yaw - current_yaw) > 180):
            if(destination_yaw > current_yaw):
                temp_desired_yaw = destination_yaw - current_yaw
                temp_current_yaw = 361

            elif current_yaw > destination_yaw:
                temp_current_yaw = current_yaw - destination_yaw
                temp_desired_yaw = 361

            velocity_yaw_des, cached_var = MPC_solver(temp_current_yaw, temp_desired_yaw, 370, home_yaw, n, t, True, vel_limit = ang_vel_lim, variables = cached_var)

        else:
            velocity_yaw_des, cached_var = MPC_solver(current_yaw, destination_yaw, 370, home_yaw, n, t, True, vel_limit = ang_vel_lim, variables = cached_var)

        theta_arr = -(cached_var.get("points") - current_yaw)

        print current_yaw, destination_yaw, current_r, destination_r

        destination_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        destination_point.header.frame_id = 'local_origin'
        destination_point.point.x = destination_x - home_x
        destination_point.point.y = destination_y - home_y
        destination_point.point.z = 0
        pub.publish(destination_point)

        move_cmd = Twist()
        if(destination_r < 0.01):
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0

        else:
            move_cmd.linear.x = velocity_r_des
            move_cmd.angular.z = -velocity_yaw_des
        pub2.publish(move_cmd)

        boundary_cube = Marker()
        boundary_cube.header.frame_id = 'local_origin'
        boundary_cube.header.stamp = rospy.Time.now()
        boundary_cube.action = boundary_cube.ADD
        boundary_cube.type = boundary_cube.CUBE
        boundary_cube.id = 0
        boundary_cube.scale.x = limit_x*2
        boundary_cube.scale.y = limit_y*2
        boundary_cube.scale.z = 1       
        boundary_cube.color.a = 0.5 
        boundary_cube.color.r = 0.4
        boundary_cube.color.g = 0.2
        boundary_cube.color.b = 0
        pub3.publish(boundary_cube)

        if True:
            mpc_pose_array = [None] * n
            for i in range(0, n):
                mpc_pose = PoseStamped()
                mpc_pose.header.seq = i
                mpc_pose.header.stamp = rospy.Time.now() + rospy.Duration(t * i)
                mpc_pose.header.frame_id = "local_origin"
                mpc_pose.pose.position.x = r_arr[i] * math.cos(theta_arr[i] * 3.1416/180) + destination_x
                mpc_pose.pose.position.y = r_arr[i] * math.sin(theta_arr[i] * 3.1416/180) + destination_y
                mpc_pose_array[i] = mpc_pose

        if (xAnt != mpc_pose.pose.position.x and yAnt != mpc_pose.pose.position.y):
            # mpc_pose.header.seq = ekf_path.header.seq + 1
            mpc_path.header.frame_id = "local_origin"
            mpc_path.header.stamp = rospy.Time.now()
            # mpc_pose.header.stamp = mpc_path.header.stamp
            mpc_path.poses = mpc_pose_array
            # cont = cont + 1

        xAnt = mpc_pose.pose.orientation.x
        yAnt = mpc_pose.pose.position.y

        pub4.publish(mpc_path)

        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "odom", "local_origin")
        sys.stdin.flush()



if __name__ == "__main__":
    path = Path() 
    # np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
    main()