#!/usr/bin/python

from __future__ import print_function

import os, sys, select, tty, rospy, mavros, threading, time, signal, select, tf, math, quadprog, qp_matrix, argparse, termios

from mavros import command
from mavros import setpoint as SP

from std_msgs.msg import Header, Float32, Float64, Empty, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
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
from qp_matrix import qp_q_dot_des_array
from MPC_obstacle_avoidance import MPC_solver

global R, args, uav_x_vel, uav_y_vel

n                                                   = 15
t                                                   = 0.1
height                                              = 3
lin_vel_lim                                         = .5
kp                                                  = 1.
kb                                                  = 10000000.0
gps_rate                                            = 0
cont                                                = 0
x_home_recorded = z_home_recorded                   = False
x_current = y_current = z_current                   = 0.0
x_home = y_home = z_home                            = 0.0
ekf_x = ekf_y = ekf_z                               = 0
x_destination = y_destination =  z_destination      = 0.01
roll = pitch = yaw                                  = 0.0
TIMEOUT                                             = 0.5
y_homeyaw                                           = 0
br                                                  = tf.TransformBroadcaster()
br2                                                 = tf.TransformBroadcaster()
y_pub                                               = rospy.Publisher('y_graph', Float32, queue_size = 5)
discard_samples                                     = 20                        #samples to discard before gps normalizes
pos                                                 = Point()
quat                                                = Quaternion()
quat.w                                              = 1
start_y                                             = 0.0
cached_var                                          = {}
x_obs = y_obs = r_obs                               = [0.0]
vx_obs                                              = [0.0]
vy_obs                                              = [0.0]
other_drone                                           = [[],[],[],[],[]]
r_vehicle                                           = 0.5
final_pose                                          = [0, 0, 0]
init_pose                                           = [0, 0, 0]
current_pose                                        = [0, 0, 0]
UAV_state                                           = State()
name_index                                          = 1
prev_models_length                                  = 0
dest_received_flag                                  = False
x_temp = y_temp                                     = 0.
counter                                             = 0
x_seperation = y_seperation                         = 0.
has_reached                                         = False
is_leader                                           = False
is_switch_active                                    = False
# last_time                                           = rospy.rostime.Time()

# def hover_callback(event):
#     global pub
#     if event.current_real-last_time>rospy.Duration(0.1):
#         print("hover")
#         pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

def dead_man_switch(data):
    global is_switch_active
    is_switch_active = data

def home_pos_cb(data):
    global x_home, y_home, z_home
    # x_home = data.geo.latitude
    # y_home = data.geo.longitude
    # z_home = data.geo.altitude

def other_drone_positions_cb(data):
    global other_drone

    other_drone = [[data.center.x], [data.center.y], [data.radius], [data.velocity.x], [data.velocity.y]]
    # print(len(other_drone), other_drone[0], other_drone[1])

def gps_local_cb(data):
    global x_current, y_current, z_current, x_home, y_home, z_home, z_destination, x_home_recorded, discard_samples, x_destination, y_destination, start_y, pos

    x_current = data.pose.pose.position.x + x_seperation
    y_current = data.pose.pose.position.y + y_seperation
    z_current = data.pose.pose.position.z

    pos.x = x_current
    pos.y = y_current
    pos.z = z_current
    if x_home_recorded is False and x_current != 0 and y_current != 0:
        x_home = x_current
        y_home = y_current
        discard_samples = discard_samples - 1

        if(discard_samples <= 0):
            x_destination = x_current                       #to set home position as initial desired position
            y_destination = y_current
            z_destination = z_current + height
            z_home = z_current
            # z_home_recorded = True
            start_y = y_home
            x_home_recorded = True


def pose_cb(data):
    global ekf_x, ekf_y,ekf_z
    position = data.pose.position
    ekf_x = position.x
    ekf_y = position.y
    ekf_z = position.z

def calc_target_cb(data):
    global x_destination, y_destination, z_destination, final_pose, seperation
    print('Here')

    #Subtract seperation of 2nd drone and 1st drone from target point
    x_destination = data.pose.position.x# - x_seperation
    y_destination = data.pose.position.y# - y_seperation

    my_dist_to_target = math.sqrt((x_destination - x_current) ** 2 + (y_destination - y_current) ** 2)
    other_dist_to_target = math.sqrt((x_destination - other_drone[0][0]) ** 2 + (y_destination - other_drone[1][0]) ** 2)
    print(x_seperation, y_seperation)

    # if(my_dist_to_target < other_dist_to_target):
    final_pose = [x_destination, y_destination, height]
    print(args.namespace, final_pose)

def twist_obj(x, y, z, a, b, c):
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
    global pos, quat, args, name_index, prev_models_length, final_pose, height, z_destination
    if(args.namespace is not ''):
        all_models_length = len(data.name)

        if(name_index == 0 or all_models_length != prev_models_length):
            name_index = data.name.index(args.namespace, len(data.name)-3, len(data.name))
            prev_models_length = all_models_length

    if(args.gripped_object is not ''):
        all_models_length = len(data.name)

        if(name_index == 0 or all_models_length != prev_models_length):
            name_index = data.name.index(args.gripped_object, 0, len(data.name))
            x_destination = data.pose[name_index].position.x
            y_destination = data.pose[name_index].position.y
            final_pose = [x_destination, y_destination, height]
            # z_destination = z_destination + float(args.flight_height)

    # print(name_index)
    pos = data.pose[name_index].position
    quat = data.pose[name_index].orientation

def drone_vel_cb(data):
    global uav_x_vel, uav_y_vel
    uav_x_vel = data.twist.linear.x
    uav_y_vel = data.twist.linear.y

def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided

def _setpoint_position_callback(topic):
    pass

def main():
    global x_home_recorded, z_home_recorded, args, counter, lin_vel_lim, z_destination, x_seperation, y_seperation, pub
    global kp, kb, cont, gps_rate, n, t, cached_var, other_drone, x_velocity_des, y_velocity_des, is_leader, last_time, height

    settings = termios.tcgetattr(sys.stdin)
    xAnt = yAnt = 0
    x_home_recorded = False
    max_time = 0.
    min_time = 1000.
    total_time = 0.
    
    path = Path() 
    height = float(args.flight_height)
    rate = rospy.Rate(50.0)

    if(gps_rate == 0):
        rospy.Subscriber(args.namespace+'/mavros/global_position/local', Odometry, gps_local_cb)

    elif(gps_rate == 1):    
        rospy.Subscriber('/global_position_slow', Odometry, gps_local_cb)

    else:
        gps_rate = 0

    # rospy.Timer(rospy.Duration(0.1), hover_callback)
    if(not args.individual):
        rospy.Subscriber(args.other_namespace+'/current_pose', CircleObstacle, other_drone_positions_cb)
    rospy.Subscriber(mavros.get_topic('altitude'), Altitude, alt_cb)
    rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, pose_cb)
    rospy.Subscriber('/gazebo/model_states', ModelStates, gazebo_cb)
    rospy.Subscriber(mavros.get_topic('local_position', 'velocity'), TwistStamped, drone_vel_cb)
    rospy.Subscriber('dead_man_switch', Bool, dead_man_switch)
    this_drone_coords = rospy.wait_for_message(mavros.get_topic('global_position', 'global'), NavSatFix, timeout=2)

    try:
        if(rospy.wait_for_message(args.other_namespace+'/current_pose', CircleObstacle, timeout=.2)):
            print('Waiting')
            other_drone_coords = rospy.wait_for_message(args.other_namespace+'/mavros/global_position/global', NavSatFix, timeout=2)
            R = 6371e3 # metres
            lat1 = other_drone_coords.latitude
            lon1 = other_drone_coords.longitude
            lat2 = this_drone_coords.latitude
            lon2 = this_drone_coords.longitude
            phi_1 = math.radians(lat1)
            phi_2 = math.radians(lat2)
            delta_phi = math.radians(lat2-lat1)
            delta_lambda = math.radians(lon2-lon1)

            a = math.sin(delta_phi/2) * math.sin(delta_phi/2) + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda/2) * math.sin(delta_lambda/2)
            
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

            seperation = R * c

            y = math.sin(math.radians(lon2-lon1)) * math.cos(phi_2)
            x = math.cos(phi_1)*math.sin(phi_2) - math.sin(phi_1)*math.cos(phi_2)*math.cos(math.radians(lon2-lon1))
            brng = math.degrees(math.atan2(y, x))
            x_seperation = seperation * math.sin(math.radians(brng))
            y_seperation = seperation * math.cos(math.radians(brng))

            print(seperation, x_seperation, y_seperation)

    except:
        #If controlling 1st drone, create topic to publish target point for another drone
        point_pub = rospy.Publisher(args.namespace + '/go_to_point', PoseStamped, queue_size=1)
        is_leader = True
        print('First drone')
        
    #Subscribe to target topic published by another drone if exists, else listen to rViz
    if 'point_pub' in locals():
        print("Listening to rViz")
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, calc_target_cb)

    else:
        print("Listening to 1st drone")
        rospy.Subscriber(args.other_namespace+'/go_to_point', PoseStamped, calc_target_cb)

    pub = rospy.Publisher(args.namespace+'destination_point', PointStamped, queue_size = 1)
    pub1 = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, queue_size = 3)  
    pub2 = rospy.Publisher(args.namespace+'/gps_point', PointStamped, queue_size = 5)
    pub3 = rospy.Publisher('boundary_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher('path', Path, queue_size=1)
    pub5 = rospy.Publisher(args.namespace + '/home_pos', HomePosition, queue_size=5)
    pub6 = rospy.Publisher(args.namespace + '/mpc_path', Path, queue_size=1)
    pub7 = rospy.Publisher(args.namespace + '/current_pose', CircleObstacle, queue_size=3)
    pub8 = rospy.Publisher(args.namespace + '/predicted_path', Path, queue_size = 1)

    path = Path()
    mpc_path = Path()
    obs_path = Path()
    max_append = 1000

    # setup service
    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/'+args.namespace+'/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/'+args.namespace+'/mavros/set_mode', mavros_msgs.srv.SetMode)
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'), mavros_msgs.msg.PositionTarget, _setpoint_position_callback)

    setpoint_msg = mavros.setpoint.TwistStamped(
        header=mavros.setpoint.Header(
        stamp=rospy.Time.now()),
    )
    
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
    
    while not rospy.is_shutdown():
        if (UAV_state.mode != 'OFFBOARD' and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            set_mode(0, 'OFFBOARD')
            print('enabling offboard mode')
            last_request = rospy.Time.now()
        else:
            if (not UAV_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if (mavros.command.arming(True)):
                    print('Vehicle armed')
                last_request = rospy.Time.now()
 
        timer = time.time()
        tty.setraw(sys.stdin.fileno())
        ready = select.select([sys.stdin], [], [], 0.025)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        if ready == ([sys.stdin], [], []):
            x = sys.stdin.read(1)
            sys.stdin.flush()

            if x == 'p':
                kp = float(raw_input('Enter kp limit:'))

            if x == 'b':
                kb = float(raw_input('Enter kb limit:'))

            # if x == 's':
            #     gps_rate = int(raw_input('0 - original, 1 - slow:'))

            if x == 'n':
                n = int(raw_input('Enter nsteps:'))

            if x == 't':
                t = float(raw_input('Enter timestep duration:'))

            if x == 'l':
                lin_vel_lim = float(raw_input('Enter linear velocity limit:'))

            if x == 'w':
                if(height < 2):
                    height = height + 0.5

            if x == 's':
                if(height > 0.4):
                    height = height - 0.2

                else:
                    height = 0.4

            sys.stdin.flush()

        dx = x_current - final_pose[0]
        dy = y_current - final_pose[1]
        dz = height - z_current
        current_pose = [dx, dy, dz]

        timer = time.time()

        # other_drone[3] = [uav_x_vel]
        # other_drone[4] = [uav_y_vel]
        try:
            x_velocity_des, y_velocity_des, cached_var = MPC_solver(init_pose, current_pose, final_pose, nsteps=n, interval=t, variables=cached_var, r_vehicle=r_vehicle, obstacles=other_drone)

        except ValueError:
            x_velocity_des = 0
            y_velocity_des = 0

        # current_time = time.time() - timer

        # if(current_time > max_time):
        #     max_time = current_time

        # if(current_time < min_time):
        #     min_time = current_time

        # total_time += current_time
        # counter = counter + 1
        # avg_time = total_time / counter

        # if(counter > 100000):
        #     total_time = 0.
        #     counter = 0
        # print('Average time = %f \t Max time = %f \t Min time = %f' % (avg_time, max_time, min_time))
        # print(time.time() - timer)

        x_array = cached_var.get('solution')[1:n+1]
        y_array = cached_var.get('solution')[2 * n + 2:2 * n + 1 + n + 1]

        z_velocity_des = kp * dz

        mpc_point_arr = np.transpose(np.row_stack((x_array, y_array)))

        theta = math.atan2(y_velocity_des, x_velocity_des)
        x_vel_limit = lin_vel_lim * math.cos(theta)
        y_vel_limit = lin_vel_lim * math.sin(theta)

        x_velocity_des = x_velocity_des if math.fabs(x_velocity_des) < x_vel_limit else x_vel_limit# * math.copysign(1, x_velocity_des)
        y_velocity_des = y_velocity_des if math.fabs(y_velocity_des) < y_vel_limit else y_vel_limit# * math.copysign(1, y_velocity_des)

        # if(is_switch_active):
        pub1.publish(twist_obj(x_velocity_des, y_velocity_des, z_velocity_des, 0.0, 0.0, 0.0))

        # else:
        #     pub1.publish(twist_obj(0, 0, 0, 0, 0, 0))
        
        # print (x_current, y_current, x_home, y_home, x_velocity_des, y_velocity_des, z_velocity_des)
        # print((x_current - x_home), (y_current - y_home), x_destination - x_home, y_destination - y_home)
        # print(x_destination, y_destination)
        print(height, dz)
        desired_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        desired_point.header.frame_id = 'local_origin'
        desired_point.point.x = final_pose[0]
        desired_point.point.y = final_pose[1]
        desired_point.point.z = 0
        pub.publish(desired_point)

        gps_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        gps_point.header.frame_id = 'local_origin'
        gps_point.point.x = x_current
        gps_point.point.y = y_current
        gps_point.point.z = z_current
        pub2.publish(gps_point)

        pose = PoseStamped()
        pose.header.frame_id = 'local_origin'
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z

        if True:
            mpc_pose_array = [None] * n
            for i in range(0, n):
                mpc_pose = PoseStamped()
                mpc_pose.header.seq = i
                mpc_pose.header.stamp = rospy.Time.now() + rospy.Duration(t * 1)
                mpc_pose.header.frame_id = 'local_origin'
                mpc_pose.pose.position.x = mpc_point_arr[i][0] + final_pose[0]
                mpc_pose.pose.position.y = mpc_point_arr[i][1] + final_pose[1]
                mpc_pose.pose.position.z = height
                mpc_pose_array[i] = mpc_pose

        # if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
        pose.header.seq = path.header.seq + 1
        path.header.frame_id = 'local_origin'
        path.header.stamp = rospy.Time.now()
        pose.header.stamp = path.header.stamp
        path.poses.append(pose)
        
        mpc_path.header.frame_id = 'local_origin'
        mpc_path.header.stamp = rospy.Time.now()
        mpc_path.poses = mpc_pose_array
        cont = cont + 1

        current_position = CircleObstacle()
        current_position.center = pos
        current_position.radius = .5
        current_position.true_radius = 1
        current_position.velocity.x = uav_x_vel
        current_position.velocity.y = uav_y_vel

        if len(other_drone) > 0 and len(other_drone[0]) > 0:
            obs_pose_array = [None] * 2

            for i in range(0, 2):
                obs_pose = PoseStamped()
                obs_pose.header.seq = i
                obs_pose.header.stamp = rospy.Time.now() + rospy.Duration(t * n * i)
                obs_pose.header.frame_id = 'local_origin'
                
                obs_pose.pose.position.x = other_drone[0][0] + (other_drone[3][0] * n * t * i)
                obs_pose.pose.position.y = other_drone[1][0] + (other_drone[4][0] * n * t * i)
                obs_pose_array[i] = obs_pose

            obs_path.header.frame_id = 'local_origin'
            obs_path.header.stamp = rospy.Time.now()
            obs_path.poses = obs_pose_array

        pub4.publish(path)
        pub6.publish(mpc_path)
        pub7.publish(current_position)
        pub8.publish(obs_path)

        if cont > max_append and len(path.poses) != 0:
            path.poses.pop(0)

        if(is_leader):
            if(math.fabs(x_current - final_pose[0]) < 0.5 and math.fabs(y_current - final_pose[1]) < 0.5):
                other_drone_target = PoseStamped(header=Header(stamp=rospy.get_rostime()))
                other_drone_target.pose.position.x = x_current + 3
                other_drone_target.pose.position.y = y_current
                point_pub.publish(other_drone_target)
                # print("Reached destination, sending co-ordinates")

        br.sendTransform((pos.x, pos.y, pos.z), (quat.x, quat.y, quat.z, quat.w), rospy.Time.now(), args.namespace+'/base_link', 'local_origin')
        # br2.sendTransform((pos.x, pos.y, pos.z), (0, 0, 0, 1), rospy.Time.now(), 'base_scan', 'local_origin')
        last_time = rospy.Time.now()

        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--namespace', default='', help='Specify namespace for individual drone')
    parser.add_argument('-o', '--other_namespace', default='', help='Specify namespace of other drone')
    parser.add_argument('-g', '--gripped_object', default='unit_box', help='Specify object to be gripped')
    parser.add_argument('-f', '--flight_height', default='2', help='Height at which drone should fly')
    parser.add_argument('-i', '--individual', action='store_true')
    args = parser.parse_args()

    mavros.set_namespace(args.namespace+'/mavros')
    
    rospy.init_node(args.namespace+'_controller')

    main()