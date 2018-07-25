#!/usr/bin/python
import os
import rospy
import roslib
import select
import sys
import tf

from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
import quadprog
import numpy as np
from numpy import array
import math
from qp_matrix import qp_q_dot_des_array
from MPC_obstacle_avoidance import MPC_solver
import qp_matrix
from tf.transformations import euler_from_quaternion

current_x = current_y = current_yaw = 0.0
destination_x = destination_y = destination_yaw = 0.0
home_y =  home_yaw = 0
home_x = 0
limit_x = limit_y = 200
yaw = 0
home_yaw_recorded = False
n = 30
t = .1
br = tf.TransformBroadcaster()
br2 = tf.TransformBroadcaster()
cached_var = {}
lin_vel_lim = .08
ang_vel_lim = .4
disp_obs_x = [1., 1.5, 2.]
disp_obs_y = [0., -0.5, 0.]
obs_x = np.array(disp_obs_x) * 0.95
obs_y = np.array(disp_obs_y) * 0.95
obs_r = [.14, .125, 0.125]
obstacles = [obs_x, obs_y, obs_r]
vehicle_r = .16
batt_low = False
scale_factor = 1.0

def odom_cb(data):
    global current_yaw, current_x, current_y

    current_x = -data.pose.pose.position.x
    current_y = -data.pose.pose.position.y
    orientation_q = data.pose.pose.orientation
    orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    (roll, pitch, current_yaw) = euler_from_quaternion(orientation_list)
    (roll, pitch, current_yaw) = (0, 0, current_yaw  * 180.0/3.1416)


def calc_target_cb(data):
    global destination_x, destination_y, desired_z
    destination_x = home_x + data.pose.position.x
    destination_y = home_y + data.pose.position.y

def batt_voltage_cb(data):
    
    if data.voltage < 10.0:
        print("Battery critical")
        batt_low = True

    elif(data.voltage < 11.0):
        print("Battery low")


def main():
    global home_yaw, home_yaw_recorded, home_x, home_y, current_x, current_y, current_yaw, limit_x, limit_y, n, t, cached_var, ang_vel_lim, lin_vel_lim
    global limit_x, limit_y
    xAnt = yAnt = 0

    rospy.init_node('MAVROS_Listener')
    
    rate = rospy.Rate(10.0)

    rospy.Subscriber("/odom_rf2o", Odometry, odom_cb)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)
    rospy.Subscriber("/battery_state", BatteryState, batt_voltage_cb)

    pub = rospy.Publisher('destination_point', PointStamped, queue_size = 1)
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
    pub3 = rospy.Publisher('boundary_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher('mpc_path', Path, queue_size=1)
    pub5 = rospy.Publisher('obs_point_1', PointStamped, queue_size = 1)
    pub7 = rospy.Publisher('obs_point_2', PointStamped, queue_size = 1)
    pub8 = rospy.Publisher('obs_point_3', PointStamped, queue_size = 1)
    pub6 = rospy.Publisher('turtle_point', PointStamped, queue_size = 1)
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
       
        # current_r = math.sqrt(current_x * current_x + current_y * current_y)
        destination_r = math.sqrt(math.pow(destination_x - current_x, 2) + math.pow(destination_y - current_y, 2))
        # limit_r = math.sqrt(limit_x * limit_x + limit_y * limit_y)

        
        # velocity_r_des, cached_var = MPC_solver(current_r * 0, destination_r, limit_r, home_x, n, t, True, variables = cached_var, vel_limit = lin_vel_lim)
        # r_arr = cached_var.get("points")
        # velocity_y_des, cached_var = MPC_solver(current_y, destination_y, limit_y, home_y, n, t, True, variables = cached_var, vel_limit = lin_vel_lim)

        # this is for controlling the turtle bot, mpc solver only yields paths in cartesian space.
        dx = destination_x * scale_factor - current_x
        dy = destination_y * scale_factor - current_y

        try:
            velocity_x_des, velocity_y_des, cached_var = MPC_solver(r_actual=dx, r_desired=0., r_destination = destination_x*scale_factor, r_origin=home_x,t_actual=dy,t_desired=0,t_destination=destination_y*scale_factor,t_origin=home_y,nsteps=n,interval=t, variables=cached_var, vehicle_r=vehicle_r, obstacles=obstacles)

        except ValueError:
            velocity_x_des = 0
            velocity_y_des = 0
            
        x_arr = cached_var.get("solution")[1:n+1]
        y_arr = cached_var.get("solution")[2 * n + 2:2 * n + 1 + n + 1]

        velocity_x_des = np.clip(velocity_x_des, -lin_vel_lim, lin_vel_lim)
        velocity_y_des = np.clip(velocity_y_des, -lin_vel_lim, lin_vel_lim)

        x_e = x_arr[1] - x_arr[0]
        y_e = y_arr[1] - y_arr[0]
        # theta_e = 

        destination_yaw  = math.atan2(y_e, x_e) * 180 / 3.1416
        current_yaw = 360.0 + current_yaw if current_yaw < 0 else current_yaw
        destination_yaw = 360.0 + destination_yaw if destination_yaw < 0 else destination_yaw
        if(math.fabs(destination_yaw - current_yaw) > 180):
            if(destination_yaw > current_yaw):
                temp_desired_yaw = destination_yaw - current_yaw
                temp_current_yaw = 361

            elif current_yaw > destination_yaw:
                temp_current_yaw = current_yaw - destination_yaw
                temp_desired_yaw = 361

            velocity_yaw_des = np.clip(temp_desired_yaw - temp_current_yaw, -ang_vel_lim, ang_vel_lim); 
            # velocity_r_des, velocity_yaw_des, cached_var = MPC_solver(0, destination_r, limit_r, home_x, temp_current_yaw, temp_desired_yaw, home_yaw, n, t, lin_vel_limit = lin_vel_lim, ang_vel_limit = ang_vel_lim, variables=cached_var)
            # velocity_yaw_des, cached_var = MPC_solver(temp_current_yaw, temp_desired_yaw, 370, home_yaw, n, t, True, vel_limit = ang_vel_lim, variables = cached_var)

        else:
            velocity_yaw_des = np.clip(destination_yaw - current_yaw, -ang_vel_lim, ang_vel_lim);
            # velocity_r_des, velocity_yaw_des, cached_var = MPC_solver(0, destination_r, limit_r, home_x, current_yaw, destination_yaw, home_yaw, n, t, lin_vel_limit = lin_vel_lim, ang_vel_limit = ang_vel_lim, variables=cached_var)
            # velocity_yaw_des, cached_var = MPC_solver(current_yaw, destination_yaw, 370, home_yaw, n, t, True, vel_limit = ang_vel_lim, variables = cached_var)

        # theta_arr = -(cached_var.get("points") - current_yaw)
        # print(velocity_yaw_des)

        destination_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        destination_point.header.frame_id = 'local_origin'
        destination_point.point.x = destination_x - home_x
        destination_point.point.y = destination_y - home_y
        destination_point.point.z = 0
        pub.publish(destination_point)

        obs_point_1 = PointStamped(header=Header(stamp=rospy.get_rostime()))
        obs_point_1.header.frame_id = 'local_origin'
        obs_point_1.point.x = disp_obs_x[0]
        obs_point_1.point.y = disp_obs_y[0]
        obs_point_1.point.z = 0
        pub5.publish(obs_point_1)

        turtle_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        turtle_point.header.frame_id = 'local_origin'
        turtle_point.point.x = current_x 
        turtle_point.point.y = current_y
        turtle_point.point.z = 0
        pub6.publish(turtle_point)

        obs_point_2 = PointStamped(header=Header(stamp=rospy.get_rostime()))
        obs_point_2.header.frame_id = 'local_origin'
        obs_point_2.point.x = disp_obs_x[1] 
        obs_point_2.point.y = disp_obs_y[1]
        obs_point_2.point.z = 0
        pub7.publish(obs_point_2)

        obs_point_3 = PointStamped(header=Header(stamp=rospy.get_rostime()))
        obs_point_3.header.frame_id = 'local_origin'
        obs_point_3.point.x = disp_obs_x[2] 
        obs_point_3.point.y = disp_obs_y[2]
        obs_point_3.point.z = 0
        pub8.publish(obs_point_3)


        move_cmd = Twist()
        if(destination_r < 0.1):
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0

        else:
            # move_cmd.linear.x = 0

            move_cmd.linear.x = math.sqrt(velocity_x_des**2 + velocity_y_des**2)
            move_cmd.angular.z = -velocity_yaw_des
            # -(destination_yaw - current_yaw) * 0.001
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
                # mpc_pose.pose.position.x = r_arr[i] * math.cos(theta_arr[i] * 3.1416/180) + destination_x
                mpc_pose.pose.position.x =  - x_arr[i] + destination_x
                # mpc_pose.pose.position.y = r_arr[i] * math.sin(theta_arr[i] * 3.1416/180) + destination_y
                mpc_pose.pose.position.y = - y_arr[i] + destination_y
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
        # br2.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "odom", "base_link")
        sys.stdin.flush()



if __name__ == "__main__":
    path = Path() 
    np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
    main()