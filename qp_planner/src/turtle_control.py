#!/usr/bin/python
import os, rospy, roslib, select, sys, tf, time, math, qp_matrix, quadprog

from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates, ModelState
import numpy as np
from numpy import array
from qp_matrix import qp_q_dot_des_array
from MPC_obstacle_avoidance import MPC_solver
from tf.transformations import euler_from_quaternion
from qp_planner.msg import algomsg
from qp_planner.msg import Obstacles

current_x = current_y = current_yaw = 0.0
destination_x = destination_y = destination_yaw = 0.0
final_pose = [0, 0, 0]
init_pose = [0, 0, 0]
current_pose = [0, 0, 0]
home_y =  home_yaw = 0
home_x = 0
limit_x = limit_y = 200
yaw = 0
home_yaw_recorded = False
n = 20
t = .1
br = tf.TransformBroadcaster()
br2 = tf.TransformBroadcaster()
pub6 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
cached_var = {}
lin_vel_lim = .1
ang_vel_lim = .4

obs_x = obs_y = obs_r = []
obs_v = 0.
obstacles = []
vehicle_r = 0.1
batt_low = False
scale_factor = 1.0
is_simulation = 1              #-1 for real, 1 for simulation

counter = 0
max_time = 0.
min_time = 1000.
total_time = 0.

def model_state_cb(data):
    obstacle = ModelState()

    # index_of_interest = -1
    # for i in range(len(data.name)):
    #     if data.model_name[i] == 'unit_cylinder':
    #         index_of_interest = i
    #         break

    obstacle.model_name = "unit_cylinder"
    obstacle.pose = data.pose[2]
    obstacle.twist.linear.x = 0.1
    obstacle.reference_frame = "world"
    pub6.publish(obstacle)

#Odometry data callback
def odom_cb(data):
    global current_yaw, current_x, current_y, current_pose

    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    orientation_q = data.pose.pose.orientation
    orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    (roll, pitch, current_yaw) = euler_from_quaternion(orientation_list)
    (roll, pitch, current_yaw) = (0, 0, current_yaw  * 180.0/3.1416)

#Callback for setting destination for the solver
def calc_target_cb(data):
    global destination_x, destination_y, final_pose
    destination_x = home_x + data.pose.position.x
    destination_y = home_y + data.pose.position.y
    final_pose = [destination_x,  destination_y, 0]

#To check for battery voltage of the Turlebot
def batt_voltage_cb(data):
    
    if data.voltage < 10.0:
        print("Battery critical")
        batt_low = True

    elif(data.voltage < 11.0):
        print("Battery low")

#Receives obstacles info(x, y, radius) from the topic
def obstacles_cb(data):
    global obs_x, obs_y, obs_r, obstacles

    if(data != None):
        obs_x = np.array([obj.center.x for obj in data.circles])
        obs_y = np.array([obj.center.y for obj in data.circles])
        obs_r = np.array([obj.radius for obj in data.circles])
        obstacles = [obs_x, obs_y, obs_r]

def main():
    global home_yaw, home_yaw_recorded, home_x, home_y, current_x, current_y, current_yaw, limit_x, limit_y, n, t, cached_var, ang_vel_lim, lin_vel_lim
    global limit_x, limit_y, obstacles, obs_x, obs_y, obs_r, max_time, min_time, counter, total_time
    xAnt = yAnt = 0

    rospy.init_node('Turtle_Controller')

    rate = rospy.Rate(10.0)

    #Lidar odometry
    # rospy.Subscriber("/odom_rf2o", Odometry, odom_cb)
    #Wheel encoder odometry
    rospy.Subscriber("/odom", Odometry, odom_cb)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)
    rospy.Subscriber("/battery_state", BatteryState, batt_voltage_cb)
    rospy.Subscriber("/raw_obstacles", Obstacles, obstacles_cb)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_cb)

    pub = rospy.Publisher('destination_point', PointStamped, queue_size = 1)
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
    pub3 = rospy.Publisher('boundary_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher('mpc_path', Path, queue_size=1)
    pub5 = rospy.Publisher('turtle_point', PointStamped, queue_size = 1)
    
    mpc_path = Path()

    while not rospy.is_shutdown():
        if home_yaw_recorded is False and current_yaw != 0:
            home_yaw = current_yaw
            home_yaw_recorded = True

        ready = select.select([sys.stdin], [], [], 0)[0]

        if ready:
            x = ready[0].read(1)

            if x == 'x':
                sys.stdin.flush()
                limit_x = float(raw_input('Enter x limit:'))

            if x == 'y':
                sys.stdin.flush()
                limit_y = float(raw_input('Enter y limit:'))

            if x == 'n':
                sys.stdin.flush()
                n = int(raw_input("Enter nsteps:"))

            if x == 't':
                sys.stdin.flush()
                t = float(raw_input("Enter timestep duration:"))
       
        # current_r = math.sqrt(current_x * current_x + current_y * current_y)
        destination_r = math.sqrt(math.pow(destination_x - current_x, 2) + math.pow(destination_y - current_y, 2))
        # limit_r = math.sqrt(limit_x * limit_x + limit_y * limit_y)

        # this is for controlling the turtle bot, mpc solver only yields paths in cartesian space.
        dx = destination_x - current_x
        dy = destination_y - current_y
        current_pose = [dx, dy, 0]

        timer = time.time()
        #Calls to the MPC solver
        try:
            velocity_x_des, velocity_y_des, cached_var = MPC_solver(init_pose, current_pose, final_pose, nsteps=n, interval=t, variables=cached_var, vehicle_r=vehicle_r, obstacles=obstacles)

        except ValueError:
            velocity_x_des = 0
            velocity_y_des = 0

        current_time = time.time() - timer

        if(current_time > max_time):
            max_time = current_time

        if(current_time < min_time):
            min_time = current_time

        total_time += current_time
        counter = counter + 1
        avg_time = total_time / counter

        if(counter > 100000):
            total_time = 0.
            counter = 0
        print "Average time = %f \t Max time = %f \t Min time = %f" % (avg_time, max_time, min_time)
        # print(time.time() - timer)
            
        x_arr = cached_var.get("solution")[1:n+1]
        y_arr = cached_var.get("solution")[2 * n + 2:2 * n + 1 + n + 1]

        velocity_x_des = np.clip(velocity_x_des, -lin_vel_lim, lin_vel_lim)
        velocity_y_des = np.clip(velocity_y_des, -lin_vel_lim, lin_vel_lim)

        x_e = x_arr[1] - x_arr[0]
        y_e = y_arr[1] - y_arr[0]

        destination_yaw  = math.atan2(y_e, x_e) * 180 / 3.1416
        current_yaw = 360.0 + current_yaw if current_yaw < 0 else current_yaw
        destination_yaw = 360.0 + destination_yaw if destination_yaw < 0 else destination_yaw

        #This part controls the yaw of the turtlebot, taking into account the shortest direction for the desired yaw
        if(math.fabs(destination_yaw - current_yaw) > 180):
            if(destination_yaw > current_yaw):
                temp_desired_yaw = destination_yaw - current_yaw
                temp_current_yaw = 361

            elif current_yaw > destination_yaw:
                temp_current_yaw = current_yaw - destination_yaw
                temp_desired_yaw = 361

            velocity_yaw_des = np.clip(temp_desired_yaw - temp_current_yaw, -ang_vel_lim, ang_vel_lim); 

        else:
            velocity_yaw_des = np.clip(destination_yaw - current_yaw, -ang_vel_lim, ang_vel_lim);

        move_cmd = Twist()
        if(destination_r < 0.1):
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0

        elif(math.fabs(destination_yaw - ((current_yaw - 180) % 360)) > 20):
            move_cmd.linear.x = 0
            move_cmd.angular.z = is_simulation * velocity_yaw_des * 2

        else:
            move_cmd.linear.x = math.sqrt(velocity_x_des**2 + velocity_y_des**2)
            move_cmd.angular.z = is_simulation * velocity_yaw_des
        pub2.publish(move_cmd)

        #From hereon, deals with visualization in RViz
        destination_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        destination_point.header.frame_id = 'local_origin'
        destination_point.point.x = destination_x - home_x
        destination_point.point.y = destination_y - home_y
        destination_point.point.z = 0
        pub.publish(destination_point)

        turtle_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        turtle_point.header.frame_id = 'local_origin'
        turtle_point.point.x = current_x 
        turtle_point.point.y = current_y
        turtle_point.point.z = 0
        pub5.publish(turtle_point)

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
                mpc_pose.pose.position.x = - x_arr[i] + destination_x
                mpc_pose.pose.position.y = - y_arr[i] + destination_y
                mpc_pose_array[i] = mpc_pose

        if (xAnt != mpc_pose.pose.position.x and yAnt != mpc_pose.pose.position.y):
            mpc_path.header.frame_id = "local_origin"
            mpc_path.header.stamp = rospy.Time.now()
            mpc_path.poses = mpc_pose_array

        xAnt = mpc_pose.pose.orientation.x
        yAnt = mpc_pose.pose.position.y

        pub4.publish(mpc_path)

        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "odom", "local_origin")
        # br2.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "local_origin", "base_scan")


if __name__ == "__main__":
    np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
    main()