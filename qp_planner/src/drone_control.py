#!/usr/bin/python

from __future__ import print_function

import os, sys, select, tty, rospy, mavros, threading, time, signal, select, tf, math, quadprog, qp_matrix, argparse, tf2_ros

from mavros import command
from mavros import setpoint as SP

from std_msgs.msg import Header, Float32, Float64, Empty
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped, TransformStamped
import mavros_msgs.msg, npyscreen
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from qp_planner.msg import algomsg
from mavros_msgs.msg import Altitude, State, PositionTarget
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu, NavSatFix
from qp_planner.msg import Obstacles, CircleObstacle

from tf.transformations import euler_from_quaternion
import numpy as np
from numpy import array
from qp_matrix import qp_q_dot_des_array
from MPC_obstacle_avoidance import MPC_solver

global R, args, uav_x_vel, uav_y_vel

n                                                   = 15
t                                                   = 0.1
height                                              = 3
lin_vel_lim                                         = 2
kp                                                  = 1.
kb                                                  = 10000000.0

gps_rate                                            = 0
cont                                                = 0
x_home_recorded = z_home_recorded                   = False
x_current = y_current = z_current                   = 0.0
x_home = y_home = z_home                            = 0.0
ekf_x = ekf_y = ekf_z                               = 0
x_destination = y_destination =  z_destination      = 0.0
limit_x = limit_y = limit_z                         = 10.
roll = pitch = yaw                                  = 0.0
TIMEOUT                                             = 0.5
y_homeyaw                                           = 0
br                                                  = tf2_ros.TransformBroadcaster()
br2                                                 = tf.TransformBroadcaster()
y_pub                                               = rospy.Publisher('y_graph', Float32, queue_size = 5)
discard_samples                                     = 0                        #samples to discard before gps normalizes
pos                                                 = Point()
quat                                                = Quaternion()
start_y                                             = 0.0
timer                                               = 0.0
cached_var                                          = {}
x_obs = y_obs = r_obs                               = [0.0]
vx_obs                                              = [0.0]
vy_obs                                              = [0.0]
obstacles                                           = [[],[],[],[],[]]
r_vehicle                                           = 0.5
final_pose                                          = [0, 0, 0]
init_pose                                           = [0, 0, 0]
current_pose                                        = [0, 0, 0]
UAV_state                                           = State()
name_index                                          = 1
prev_models_length                                  = 0
dest_received_flag                                  = False
x_temp = y_temp                                     = 0.
prev_x                                              = 0.

counter                                             = 0
max_time                                            = 0.
min_time                                            = 1000.
total_time                                          = 0.
sleep_flag                                          = False

class form_object(npyscreen.Form):
    def create(self):
        global nsteps_slider, interval_slider, debug, debug2
        main_thread.start()

        nsteps_slider = self.add(npyscreen.TitleSlider, name = "No. of steps:", value = 20, out_of = 50)
        interval_slider = self.add(npyscreen.TitleSlider, name = "Time interval:", value = .2, out_of = 10, step = 0.1)
        speed_slider = self.add(npyscreen.TitleSlider, name = "Speed:", value = 0.5, out_of = 3, step = 0.1)
        debug = self.add(npyscreen.TitleText, name="Average time:")
        debug2 = self.add(npyscreen.TitleText, name="Instantaneous time:")

    def afterEditing(self):
        global main_thread
        self.parentApp.setNextForm(None)
        main_thread.do_run = False
        main_thread.join()

class App(npyscreen.NPSAppManaged):
    def onStart(self):
        self.addForm('MAIN', form_object, name = "RBCCPS ADVANCED MPC CONTROLLER v2.646.23.1")

def imu_cb(data):
    global roll, pitch, yaw
    orientation_q = data.orientation
    orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    (roll, pitch, yaw) = (roll * 180.0/3.1416, pitch * 180.0/3.1416, yaw  * 180.0/3.1416)


def obstacles_cb(data):
    global x_obs, y_obs, r_obs, obstacles, x_velocity_des, y_velocity_des

    if(data != None):
        x_obs = np.array([obj.center.x for obj in data.circles])
        y_obs = np.array([obj.center.y for obj in data.circles])
        # Using + sign for subtraction because desired velocities are negated
        # vx_obs = np.array([obj.velocity.x + (0 if x_velocity_des < 0.1 else x_velocity_des) for obj in data.circles])
        # vy_obs = np.array([obj.velocity.y + (0 if y_velocity_des < 0.1 else y_velocity_des) for obj in data.circles])
        # vx_obs = np.array([obj.velocity.x + x_velocity_des for obj in data.circles])
        # vy_obs = np.array([obj.velocity.y + y_velocity_des for obj in data.circles])
        vx_obs = np.array([obj.velocity.x for obj in data.circles])
        vy_obs = np.array([obj.velocity.y for obj in data.circles])
        r_obs = np.array([obj.radius for obj in data.circles])
        # if vx_obs[0] > 0:
        #     vx_obs[0] = 0.5

        # else:
        #     vx_obs[0] = -0.5
        # obstacles = [x_obs, y_obs, r_obs, np.zeros(len(x_obs)), np.zeros(len(x_obs))]
        obstacles = [x_obs, y_obs, r_obs, vx_obs, vy_obs]

def other_drone_positions_cb(data):
    global obstacles

    obstacles = [[data.center.x], [data.center.y], [data.radius], [data.velocity.x], [data.velocity.y]]
    # print(len(obstacles), obstacles[0], obstacles[1])

def gps_local_cb(data):
    global x_current, y_current, x_home, y_home, x_home_recorded, discard_samples, x_destination, y_destination, start_y, pos

    x_current = data.pose.position.x
    y_current = data.pose.position.y
    # z_current = data.pose.pose.position.z

    pos = data.pose.position
    quat = data.pose.orientation

    if x_home_recorded is False and x_current != 0 and y_current != 0:
        t = TransformStamped()

        t.header.stamp = rospy.Time.now() - rospy.Duration(discard_samples * .1)
        t.header.frame_id = "local_origin"
        t.child_frame_id = "base_link"
        t.transform.translation = pos
        t.transform.rotation = quat

        # br.sendTransform(t)

        x_home = x_current - pos.x
        y_home = y_current - pos.y
        discard_samples = discard_samples + 1

        if(discard_samples >= 20):
            x_destination = x_current                          #to set home position as initial desired position
            y_destination = y_current
            start_y = y_home
            x_home_recorded = True

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "local_origin"
    t.child_frame_id = "base_link"
    t.transform.translation.x = pos.x
    t.transform.translation.y = pos.y
    t.transform.translation.z = z_current
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1

    br.sendTransform(t)
    br2.sendTransform((pos.x, pos.y, pos.z), (quat.x, quat.y, quat.z, quat.w), rospy.Time.now(), "base_scan", "local_origin")


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


def desired_pos_cb(data):
    global x_temp, y_temp
    x_temp = x_home + data.x
    y_temp = y_home + data.y
    # z_destination = x_home + data.z

def calc_target_cb(data):
    global x_destination, y_destination, z_destination, final_pose

    if(args.individual):
        x_destination = x_home + data.pose.position.x
        y_destination = y_home + data.pose.position.y

    else:
        x_destination = x_temp
        y_destination = y_temp
    
    final_pose = [x_destination, y_destination, height]

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

def alt_cb(data):
    global z_current
    z_current = data.monotonic / 1

def gazebo_cb(data):
    global pos, quat, args, name_index, prev_models_length
    if(args.namespace is not ''):
        all_models_length = len(data.name)

        if(name_index == 0 or all_models_length != prev_models_length):
            name_index = data.name.index(args.namespace, len(data.name)-3, len(data.name))
            prev_models_length = all_models_length

    # print(name_index)

def drone_vel_cb(data):
    global uav_x_vel, uav_y_vel
    uav_x_vel = data.twist.linear.x
    uav_y_vel = data.twist.linear.y

def plot(vel_y):
    global y_current, y_destination, start_y

    timer = 0

    if(math.fabs(vel_y) >= 0.01):
        y_diff = Float32()
        try:
            y = math.fabs((y_current - start_y))/math.fabs((y_destination - start_y))
            if(y < 2):
                y_diff.data = y
                y_pub.publish(y_diff)

        except ZeroDivisionError as e:
            print("Oops")

        # print(y_current - start_y,y_destination - start_y, start_y,y_diff.data)

    elif(math.fabs(vel_y) < 0.01):
        start_y = y_destination

        print("-------------------Ready-------------------")

def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided

def _setpoint_position_callback(topic):
    pass

def main():
    global x_home_recorded, z_home_recorded, args, max_time, min_time, total_time, counter, lin_vel_lim, prev_x, sleep_flag
    global limit_x, limit_y, limit_z, kp, kb, cont, gps_rate, n, t, timer, cached_var, obstacles, x_velocity_des, y_velocity_des
    xAnt = yAnt = 0
    x_home_recorded = False
    
    path = Path() 

    rate = rospy.Rate(50.0)

    
    # rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
    # rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_global_cb)
    if(gps_rate == 0):
        rospy.Subscriber(args.namespace+"/mavros/local_position/pose", PoseStamped, gps_local_cb)

    elif(gps_rate == 1):    
        rospy.Subscriber("/global_position_slow", Odometry, gps_local_cb)

    else:
        gps_rate = 0

    rospy.Subscriber(args.namespace+"/mavros/altitude", Altitude, alt_cb)
    rospy.Subscriber(args.namespace+"/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_cb)
    rospy.Subscriber("/obstacles", Obstacles, obstacles_cb)
    if(not args.individual):
        rospy.Subscriber(args.other_namespace+"/current_pose", CircleObstacle, other_drone_positions_cb)
    rospy.Subscriber(mavros.get_topic("local_position", "velocity_local"), TwistStamped, drone_vel_cb)
    rospy.Subscriber(args.namespace+"/desired_pos", Point, desired_pos_cb)

    pub = rospy.Publisher('destination_point', PointStamped, queue_size = 1)
    pub1 = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, queue_size = 3)  
    pub2 = rospy.Publisher(args.namespace+'/gps_point', PointStamped, queue_size = 5)
    pub3 = rospy.Publisher('boundary_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher('path', Path, queue_size=1)
    pub5 = rospy.Publisher('ekf_path', Path, queue_size=1)
    pub6 = rospy.Publisher(args.namespace+'/mpc_path', Path, queue_size=1)
    pub7 = rospy.Publisher(args.namespace+'/current_pose', CircleObstacle, queue_size=3)
    pub8 = rospy.Publisher(args.namespace+'/predicted_path', Path, queue_size = 1)

    path = Path()
    ekf_path = Path()
    mpc_path = Path()
    obs_path = Path()
    max_append = 1000

    # setup service
    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy(args.namespace+'/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy(args.namespace+'/mavros/set_mode', mavros_msgs.srv.SetMode)
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'), mavros_msgs.msg.PositionTarget, _setpoint_position_callback)

    setpoint_msg = mavros.setpoint.TwistStamped(
        header=mavros.setpoint.Header(
        frame_id="att_pose",
        stamp=rospy.Time.now()),
    )
    
    # wait for FCU connection
    # while (not (UAV_state.connected or rospy.is_shutdown())):
    #     rate.sleep()
    
    # initialize the setpoint
    setpoint_msg.twist.linear.x = 0
    setpoint_msg.twist.linear.y = 0
    setpoint_msg.twist.linear.z = 0.1
    mavros.command.arming(True)
    
    # send 50 setpoints before starting
    for i in range(0, 50):
        pub1.publish(setpoint_msg)
        rate.sleep()
    
    set_mode(0, 'OFFBOARD')
    
    last_request = rospy.Time.now()
    
    main_thread = threading.currentThread()

    tf_buff = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buff)

    while getattr(main_thread, "do_run", True) and not rospy.is_shutdown():
        timer = time.time()

    	# yaw = 360.0 + yaw if yaw < 0 else yaw

        if z_home_recorded is False and z_current != 0:
            z_destination = z_current + height
            z_home = z_current
            z_home_recorded = True
            print(z_destination)

        ready = select.select([sys.stdin], [], [], 0)[0]

        if ready:
            x = ready[0].read(1)

            if x == 'x':
                limit_x = float(raw_input('Enter x limit:'))

            if x == 'y':
                limit_y = float(raw_input('Enter y limit:'))

            if x == 'z':
                limit_z = float(raw_input('Enter z limit:'))

            if x == 'p':
                kp = float(raw_input('Enter kp limit:'))

            if x == 'b':
                kb = float(raw_input('Enter kb limit:'))

            if x == 's':
                gps_rate = int(raw_input('0 - original, 1 - slow:'))

            if x == 'n':
                sys.stdin.flush()
                n = int(raw_input("Enter nsteps:"))

            if x == 't':
                t = float(raw_input("Enter timestep duration:"))

            if x == 'l':
                lin_vel_lim = float(raw_input("Enter linear velocity limit:"))

            sys.stdin.flush()

        if(not sleep_flag):
            rospy.sleep(2)
            sleep_flag = True

        dest_point = PointStamped(header=Header(stamp=(rospy.Time.now()), frame_id='local_origin'))
        dest_point.point.x = final_pose[0]
        dest_point.point.y = final_pose[1]
        dest_point.point.z = final_pose[2]
        p = tf_buff.transform(dest_point, "base_link", timeout=rospy.Duration(0.05))

        # dx = x_current - x_destination
        # dy = y_current - y_destination
        dx = -p.point.x
        dy = -p.point.y
        dz = z_destination - z_current
        current_pose = [dx, dy, dz]

        # if(len(obstacles) > 0):
        #     print(obstacles[0], obstacles[1])

        timer = time.time()

        # obstacles[3] = [uav_x_vel]
        # obstacles[4] = [uav_y_vel]
        try:
            x_velocity_des, y_velocity_des, cached_var = MPC_solver(init_pose, current_pose, final_pose, nsteps=n, interval=t, variables=cached_var, r_vehicle=r_vehicle, obstacles=obstacles)

        except ValueError:
            x_velocity_des = 0
            y_velocity_des = 0

        # current_time = time.time() - timer

        # if(counter > 100000 or prev_x != x_destination):
        #     #print("Counters cleared")
        #     total_time = 0.
        #     counter = 0
        #     prev_x = x_destination
        #     max_time = 0
        #     min_time = 1000

        # if(current_time > max_time):
        #      max_time = current_time

        # if(current_time < min_time):
        #     min_time = current_time

        # total_time += current_time
        # counter = counter + 1
        # avg_time = total_time / counter

        #print("Average time = %f \t Max time = %f \t Min time = %f" % (avg_time, max_time, min_time))
        #print(current_time)
        debug.value = str(avg_time)
        debug2.value = str(current_time)
        debug.display()
        debug2.display()

        x_array = cached_var.get("solution")[1:n+1]
        y_array = cached_var.get("solution")[2 * n + 2:2 * n + 1 + n + 1]

        z_velocity_des = 0.5 * dz
        
        mpc_point_arr = np.transpose(np.row_stack((x_array, y_array)))
        
        theta = math.atan2(y_velocity_des, x_velocity_des)
        x_vel_limit = lin_vel_lim * math.cos(theta)
        y_vel_limit = lin_vel_lim * math.sin(theta)
        # print(x_vel_limit, y_vel_limit, math.degrees(theta))
        x_velocity_des = x_velocity_des if math.fabs(x_velocity_des) < x_vel_limit else x_vel_limit# * math.copysign(1, x_velocity_des)
        y_velocity_des = y_velocity_des if math.fabs(y_velocity_des) < y_vel_limit else y_vel_limit# * math.copysign(1, y_velocity_des)

        pub1.publish(twist_obj(x_velocity_des, y_velocity_des, z_velocity_des, 0.0, 0.0, 0.0))
        
        # if(len(obstacles) > 0):
        #     print(current_pose, obstacles[0], obstacles[1])
        # print (x_current, y_current, x_home, y_home, x_velocity_des, y_velocity_des, z_velocity_des)
        # print((x_current - x_home), (y_current - y_home), x_destination - x_home, y_destination - y_home)
        # print(x_destination, y_destination)

        desired_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        desired_point.header.frame_id = 'base_link'
        desired_point.point.x = p.point.x - x_home
        desired_point.point.y = p.point.y - y_home
        desired_point.point.z = 0
        pub.publish(desired_point)

        gps_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        gps_point.header.frame_id = 'local_origin'
        gps_point.point.x = (x_current - x_home)
        gps_point.point.y = (y_current - y_home)
        gps_point.point.z = (z_current - z_home)
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

        # plot(y_velocity_des)

        pose = PoseStamped()
        pose.header.frame_id = "local_origin"
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z

        if True:
            mpc_pose_array = [None] * n
            for i in range(0, n):
                mpc_pose = PoseStamped()
                mpc_pose.header.seq = i
                mpc_pose.header.stamp = rospy.Time.now() + rospy.Duration(t * 1)
                mpc_pose.header.frame_id = "base_link"
                mpc_pose.pose.position.x = mpc_point_arr[i][0] + x_destination
                mpc_pose.pose.position.y = mpc_point_arr[i][1] + y_destination
                mpc_pose.pose.position.z = height
                mpc_pose_array[i] = mpc_pose

        if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
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
            # mpc_pose.header.seq = ekf_path.header.seq + 1
            mpc_path.header.frame_id = "local_origin"
            mpc_path.header.stamp = rospy.Time.now()
            # mpc_pose.header.stamp = mpc_path.header.stamp
            mpc_path.poses = mpc_pose_array
            cont = cont + 1

        xAnt = pose.pose.orientation.x
        yAnt = pose.pose.position.y

        current_position = CircleObstacle()
        current_position.center = pos
        current_position.radius = 1.5
        current_position.true_radius = 1
        current_position.velocity.x = uav_x_vel
        current_position.velocity.y = uav_y_vel

        if len(obstacles) > 0 and len(obstacles[0]) > 0:
            obs_pose_array = [None] * 2

            for i in range(0, 2):
                obs_pose = PoseStamped()
                obs_pose.header.seq = i
                obs_pose.header.stamp = rospy.Time.now() + rospy.Duration(t * n * i)
                obs_pose.header.frame_id = "local_origin"
                
                obs_pose.pose.position.x = obstacles[0][0] + (obstacles[3][0] * n * t * i)
                obs_pose.pose.position.y = obstacles[1][0] + (obstacles[4][0] * n * t * i)
                obs_pose_array[i] = obs_pose

            obs_path.header.frame_id = "local_origin"
            obs_path.header.stamp = rospy.Time.now()
            obs_path.poses = obs_pose_array

        pub4.publish(path)
        pub5.publish(ekf_path)
        pub6.publish(mpc_path)
        pub7.publish(current_position)
        pub8.publish(obs_path)

        if cont > max_append and len(path.poses) != 0 and len(ekf_path.poses):
            path.poses.pop(0)
            ekf_path.poses.pop(0)
        
if __name__ == "__main__":
    global main_thread
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--namespace", default="", help="Specify namespace for individual drone")
    parser.add_argument("-o", "--other_namespace", default="", help="Specify namespace of other drone")
    parser.add_argument("-i", "--individual", action="store_true")
    args = parser.parse_args()

    if(args.namespace == '' and args.other_namespace == ''):
        args.individual = True

    if(args.namespace == ''):
        mavros.set_namespace("mavros")

    else:
        mavros.set_namespace(args.namespace+"/mavros")
    
    np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
    rospy.init_node(args.namespace+'controller')

    # do_run = True
    # main_thread = threading.Thread(target = main)
    # app = App().run()

    main()
