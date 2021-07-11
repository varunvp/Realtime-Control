#!/usr/bin/python

from __future__ import print_function

import rospy, mavros, threading, time, tf, math, argparse, tf2_ros, npyscreen

from mavros import command, setpoint

import matplotlib.pyplot as plt
from std_msgs.msg import Header, Float32, Float64, Empty
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped, TransformStamped
import mavros_msgs.msg
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from qp_planner.msg import algomsg
from mavros_msgs.msg import Altitude, State, PositionTarget, HomePosition
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu, NavSatFix
from qp_planner.msg import Obstacles, CircleObstacle

from tf.transformations import euler_from_quaternion
import numpy as np
from numpy import array
import MPC_obstacle_avoidance
import MPC_obstacle_avoidance_cvx

global R, args, uav_x_vel, uav_y_vel, main_thread, nsteps_slider, interval_slider

n                                                   = 15
t                                                   = 0.1
height                                              = 10
lin_vel_lim                                         = .4
kp                                                  = .5
kb                                                  = 10000000.0
gps_rate                                            = 0
cont                                                = 0
x_home_recorded = z_home_recorded                   = False
x_current = y_current = z_current                   = 0.0
x_home = y_home = z_home                            = 0.0
ekf_x = ekf_y = ekf_z                               = 0
x_destination = y_destination =  z_destination      = 0.0
limit_x = limit_y = limit_z                         = 100000
roll = pitch = yaw                                  = 0.0
TIMEOUT                                             = 0.5
y_homeyaw                                           = 0
br                                                  = tf2_ros.TransformBroadcaster()
br2                                                 = tf.TransformBroadcaster()
y_pub                                               = rospy.Publisher('y_graph', Float32, queue_size = 5)
discard_samples                                     = 20                        #samples to discard before gps normalizes
pos                                                 = Point()
quat                                                = Quaternion()
start_y                                             = 0.0
timer                                               = 0.0
cached_var                                          = {}
cached_var_2                                          = {}
x_obs = y_obs = r_obs                               = []
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
gamma                                               = .0001
h_plot_1, h_plot_2, t_plot                          = [], [], []

class form_object(npyscreen.Form):
    def create(self):
        global speed_slider, nsteps_slider, interval_slider, debug, debug2
        main_thread.start()

        nsteps_slider = self.add(npyscreen.TitleSlider, name = "No. of steps:", value = 15, out_of = 50)
        interval_slider = self.add(npyscreen.TitleSlider, name = "Time interval:", value = .1, out_of = 10, step = 0.1)
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
        self.addForm('MAIN', form_object, name = "RBCCPS MPC CONTROLLER")


def other_drone_positions_cb(data):
    global obstacles, index

    # obstacles = [[data.center.x], [data.center.y], [data.radius], [data.velocity.x], [data.velocity.y]]
    x_obs = np.delete(np.array([obj.center.x for obj in data.circles]), index)
    y_obs = np.delete(np.array([obj.center.y for obj in data.circles]), index)
    vx_obs = np.delete(np.array([obj.velocity.x for obj in data.circles]), index)
    vy_obs = np.delete(np.array([obj.velocity.y for obj in data.circles]), index)
    r_obs = np.delete(np.array([obj.radius for obj in data.circles]), index)

    obstacles = [x_obs, y_obs, r_obs, vx_obs, vy_obs]
    # print(obstacles)

def gps_local_cb(data):
    global x_current, y_current, z_current, x_home, y_home, x_home_recorded, discard_samples, x_destination, y_destination, start_y, pos

    x_current = data.pose.pose.position.x
    y_current = data.pose.pose.position.y
    z_current = data.pose.pose.position.z

    pos = data.pose.pose.position
    quat = data.pose.pose.orientation

    if x_home_recorded is False and x_current != 0 and y_current != 0:
        x_home = x_current - pos.x
        y_home = y_current - pos.y
        discard_samples = discard_samples - 1

        if(discard_samples <= 0):
            x_destination = x_current                          #to set home position as initial desired position
            y_destination = y_current
            start_y = y_home
            x_home_recorded = True

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
    x_temp = data.x# + x_home
    y_temp = data.y# + y_home 
    # z_destination = x_home + data.z

def calc_target_cb(data):
    global x_destination, y_destination, z_destination, final_pose

    if(args.individual):
        x_destination = x_home + data.pose.position.x
        y_destination = y_home + data.pose.position.y
    
    else:
        x_destination = x_temp
        y_destination = y_temp
    
    x_destination = 0
    y_destination = 6
    final_pose = [x_destination, y_destination, height]
    # final_pose = [-2,-2,0]
    print(final_pose)

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
    global x_home_recorded, z_home_recorded, args, max_time, min_time, total_time, counter, lin_vel_lim, prev_x, nsteps_slider, interval_slider, obstacles
    global limit_x, limit_y, limit_z, kp, kb, cont, gps_rate, n, t, timer, cached_var, cached_var_2, obstacles, x_velocity_des, y_velocity_des, speed_slider, sleep_flag

    xAnt = yAnt = 0
    x_home_recorded = False
    
    path = Path() 

    rate = rospy.Rate(10.0)
    
    # rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_global_cb)
    if(gps_rate == 0):
        rospy.Subscriber(args.namespace+"/mavros/global_position/local", Odometry, gps_local_cb)

    elif(gps_rate == 1):    
        rospy.Subscriber("/global_position_slow", Odometry, gps_local_cb)

    else:
        gps_rate = 0

    # rospy.Subscriber(args.namespace+"/mavros/altitude", Altitude, alt_cb)
    rospy.Subscriber(args.namespace+"/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)

    if(not args.individual):
        rospy.Subscriber("/drones_state", Obstacles, other_drone_positions_cb)

    rospy.Subscriber(mavros.get_topic("local_position", "velocity_local"), TwistStamped, drone_vel_cb)
    rospy.Subscriber(args.namespace+"/desired_pos", Point, desired_pos_cb)

    pub = rospy.Publisher(args.namespace+'/destination_point', PointStamped, queue_size = 1)
    pub1 = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, queue_size = 3)  
    pub2 = rospy.Publisher(args.namespace+'/gps_point', PointStamped, queue_size = 5)
    pub3 = rospy.Publisher('boundary_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher(args.namespace+'/path', Path, queue_size=1)
    pub5 = rospy.Publisher(args.namespace+'/mpc_2_path', Path, queue_size=1)
    pub6 = rospy.Publisher(args.namespace+'/mpc_path', Path, queue_size=1)
    pub7 = rospy.Publisher(args.namespace+'/current_pose', CircleObstacle, queue_size=3)
    pub8 = rospy.Publisher(args.namespace+'/predicted_path', Path, queue_size = 1)
    pub9 = rospy.Publisher(mavros.get_topic('global_position','home'), HomePosition, queue_size = 10)

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
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'), PositionTarget, _setpoint_position_callback)

    setpoint_msg = mavros.setpoint.TwistStamped(
        header=mavros.setpoint.Header(
        frame_id="att_pose",
        stamp=rospy.Time.now()),
    )
    
    # time.sleep(1)
    #set home position
    home_latlong = HomePosition()
    home_latlong.geo.latitude = 13.027207
    home_latlong.geo.longitude = 77.563642
    home_latlong.geo.altitude = 918

    # for i in range(0, 10):
    #     pub9.publish(home_latlong)
    # initialize the setpoint
    setpoint_msg.twist.linear.x = 0
    setpoint_msg.twist.linear.y = 0
    setpoint_msg.twist.linear.z = 0.1
    
    # send 50 setpoints before starting
    for i in range(0, 10):
        pub1.publish(setpoint_msg)
        rate.sleep()
    
    set_arming(True)
    set_mode(0, 'OFFBOARD')
    last_request = rospy.Time.now()
    
    main_thread = threading.currentThread()

    tf_buff = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buff)

    while getattr(main_thread, "do_run", True) and not rospy.is_shutdown() and counter < 500:        
        n = 15 #int(nsteps_slider.value)
        t = 0.1 #interval_slider.value 
        lin_vel_lim = 0.5 # speed_slider.value
        timer = time.time()

        # yaw = 360.0 + yaw if yaw < 0 else yaw

        z_destination = 10
        z_home = z_current
        # if z_home_recorded is False and z_current != 0:
        #     z_home_recorded = True
            #print(z_destination)

        if(not sleep_flag):
            rospy.sleep(1)
            sleep_flag = True

        transform_time = rospy.Time.now()

        dx = x_current #-p.point.x
        dy = y_current #-p.point.y
        dz = z_destination - z_current
        current_pose = [dx, dy, 10]
        # obstacles = [[0],[5],[0.5],[0],[0]]

        timer = time.time()
        feasible_sol = None

        _, _, cached_var_2 = MPC_obstacle_avoidance.MPC_solver(init_pose, current_pose, final_pose, x_limit = limit_x, y_limit = limit_y, nsteps=n, interval=t, variables=cached_var_2, r_vehicle=r_vehicle, obstacles=obstacles, gamma=gamma)
        feasible_sol = cached_var_2.get("solution")

        x_array_2 = cached_var_2.get("solution")[0:n+2]
        y_array_2 = cached_var_2.get("solution")[2 * n + 1:2 * n + 1 + n + 2]

        # x_array_2 = cached_var_2.get("obs_free_sol")[0:n+2]
        # y_array_2 = cached_var_2.get("obs_free_sol")[2 * n + 1:2 * n + 1 + n + 2]

        mpc_point_arr_2 = np.transpose(np.row_stack((x_array_2, y_array_2)))

        mpc_pose_array_2 = [None] * n

        for i in range(0, n):
            mpc_pose_2 = PoseStamped()
            mpc_pose_2.header.seq = i
            mpc_pose_2.header.stamp = rospy.Time.now() + rospy.Duration(t * 1)
            mpc_pose_2.header.frame_id = args.namespace+"/base_link"
            mpc_pose_2.pose.position.x = mpc_point_arr_2[i][0] + x_destination
            mpc_pose_2.pose.position.y = mpc_point_arr_2[i][1] + y_destination
            mpc_pose_2.pose.position.z = height
            mpc_pose_array_2[i] = mpc_pose_2

        ekf_path.header.frame_id = "map"
        ekf_path.header.stamp = rospy.Time.now()
        ekf_path.poses = mpc_pose_array_2

        pub5.publish(ekf_path)

        x_velocity_des, y_velocity_des, cached_var = MPC_obstacle_avoidance_cvx.MPC_solver(init_pose, current_pose, final_pose, x_limit = limit_x, y_limit = limit_y,nsteps=n, interval=t, variables=cached_var, r_vehicle=r_vehicle, obstacles=obstacles, feasible_sol=feasible_sol, gamma=gamma)

            # print("current_pose\t", current_pose)
            # print("final_pose\t", final_pose)
            # print("obstacles\t", obstacles)

            # obs_free_sol_1 = cached_var.get("obs_free_sol")
            # obs_free_sol_2 = cached_var_2.get("obs_free_sol")

            # if(obs_free_sol_2 == obs_free_sol_1):
            # print(obs_free_sol_2)
            # print(obs_free_sol_1)

        # except ValueError:
        #     x_velocity_des = 0
        #     y_velocity_des = 0

        current_time = time.time() - timer

        if(counter > 100000 or prev_x != x_destination):
            #print("Counters cleared")
            total_time = 0.
            counter = 0
            prev_x = x_destination
            max_time = 0
            min_time = 1000

        if(current_time > max_time):
             max_time = current_time

        if(current_time < min_time):
            min_time = current_time

        total_time += current_time
        counter = counter + 1
        avg_time = total_time / counter

        print("Average time = %f \t Max time = %f \t Min time = %f" % (avg_time, max_time, min_time))
        print(current_time)
        # debug.value = str(avg_time)
        # debug2.value = str(current_time)

        # debug.display()
        # debug2.display()

        x_array = cached_var.get("solution")[0:n+1]
        y_array = cached_var.get("solution")[2 * n + 1:2 * n + 1 + n + 1]

        z_velocity_des = 0.5 * dz
        
        mpc_point_arr = np.transpose(np.row_stack((x_array, y_array)))

       
        
        theta = math.atan2(y_velocity_des, x_velocity_des)
        x_vel_limit = lin_vel_lim * math.cos(theta)
        y_vel_limit = lin_vel_lim * math.sin(theta)

        x_velocity_des = x_velocity_des if math.fabs(x_velocity_des) < x_vel_limit else x_vel_limit# * math.copysign(1, x_velocity_des)
        y_velocity_des = y_velocity_des if math.fabs(y_velocity_des) < y_vel_limit else y_vel_limit# * math.copysign(1, y_velocity_des)

        pub1.publish(twist_obj(x_velocity_des, y_velocity_des, z_velocity_des, 0.0, 0.0, 0.0))
        

        desired_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        desired_point.header.frame_id = 'map'
        desired_point.point.x = final_pose[0]# - x_home
        desired_point.point.y = final_pose[1]# - y_home
        desired_point.point.z = 0
        pub.publish(desired_point)

        if len(obstacles) > 0 and len(obstacles[0]) > 0:
            gps_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
            gps_point.header.frame_id = 'map'
            gps_point.point.x = obstacles[0][0] + (obstacles[3][0] * n * t * i)#x_current# - x_home
            gps_point.point.y = obstacles[1][0] + (obstacles[4][0] * n * t * i)#y_current# - y_home
            gps_point.point.z = z_current# - z_home
            pub2.publish(gps_point)

        boundary_cube = Marker()
        boundary_cube.header.frame_id = 'map'
        boundary_cube.header.stamp = rospy.Time.now()
        boundary_cube.action = boundary_cube.ADD
        boundary_cube.type = boundary_cube.CUBE
        boundary_cube.id = 0
        boundary_cube.scale.x = 12 #limit_x*2
        boundary_cube.scale.y = 12 #limit_y*2
        boundary_cube.scale.z = 5       
        boundary_cube.color.a = 0.1 
        boundary_cube.color.r = 0.4
        boundary_cube.color.g = 0.2
        boundary_cube.color.b = 0
        pub3.publish(boundary_cube)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z

        mpc_pose_array = [None] * n

        for i in range(0, n):
            mpc_pose = PoseStamped()
            mpc_pose.header.seq = i
            mpc_pose.header.stamp = rospy.Time.now() + rospy.Duration(t * 1)
            mpc_pose.header.frame_id = args.namespace+"/base_link"
            mpc_pose.pose.position.x = mpc_point_arr[i][0] + x_destination
            mpc_pose.pose.position.y = mpc_point_arr[i][1] + y_destination
            mpc_pose.pose.position.z = height
            mpc_pose_array[i] = mpc_pose


        if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
            # pose.header.seq = path.header.seq + 1
            # path.header.frame_id = "map"
            # path.header.stamp = rospy.Time.now()
            # pose.header.stamp = path.header.stamp
            # path.poses.append(pose)
            # ekf_pose.header.seq = ekf_path.header.seq + 1

            # mpc_pose.header.seq = ekf_path.header.seq + 1
            mpc_path.header.frame_id = "map"
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
                obs_pose.header.stamp = transform_time + rospy.Duration(t * n * i)
                obs_pose.header.frame_id = "map"
                
                obs_pose.pose.position.x = obstacles[0][0] + (obstacles[3][0] * n * t * i)
                obs_pose.pose.position.y = obstacles[1][0] + (obstacles[4][0] * n * t * i)
                obs_pose_array[i] = obs_pose

            obs_path.header.frame_id = "map"
            obs_path.header.stamp = rospy.Time.now()
            obs_path.poses = obs_pose_array

        pub4.publish(path)
        pub6.publish(mpc_path)
        pub7.publish(current_position)
        pub8.publish(obs_path)

        if cont > max_append and len(path.poses) != 0 and len(ekf_path.poses):
            path.poses.pop(0)
            ekf_path.poses.pop(0)

        if(args.plot):
            t_plot.append(total_time)
            
            obs_1 = np.array([obstacles[0][0], obstacles[1][0], 10])
            dist_1 = np.linalg.norm(np.array(current_pose) - obs_1)
            h_1 = dist_1 ** 2 - obstacles[2][0] ** 2
            h_plot_1.append(h_1)

            obs_2 = np.array([obstacles[0][1], obstacles[1][1], 10])
            dist_2 = np.linalg.norm(np.array(current_pose) - obs_2)
            h_2 = dist_2 ** 2 - obstacles[2][1] ** 2
            h_plot_2.append(h_2)

        if(abs(final_pose[1] - current_pose[1]) < 0.01):
            plt.plot(t_plot, h_plot_1)
            plt.plot(t_plot, h_plot_2)
            plt.grid()
            plt.show()

            break
        
if __name__ == "__main__":
    global main_thread, index
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--namespace", default="", help="Specify namespace for individual drone")
    parser.add_argument("-o", "--other_namespace", default="", help="Specify namespace of other drone")
    parser.add_argument("-i", "--individual", action="store_true")
    parser.add_argument("-p", "--plot", action="store_true")
    args = parser.parse_args()

    rospy.init_node(args.namespace+'controller')
    np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
    
    if(args.namespace == '' and args.other_namespace == ''):
    	args.individual = True

    if(args.namespace == ''):
        mavros.set_namespace("mavros")

    else:
        mavros.set_namespace(args.namespace+"/mavros")
    
    index = int(args.namespace[3])

    # do_run = True
    # main_thread = threading.Thread(target = main)
    # app = App().run()

    main()
