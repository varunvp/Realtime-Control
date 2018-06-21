#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import os, sys, select, termios, tty
import roslib, rospy
import argparse
import rospy
import mavros
import threading, time

from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist
from subprocess import call
from mavros_msgs.msg import OverrideRCIn
from mavros import command
from mavros import setpoint as SP
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from qp_planner.msg import algomsg


# from dynamic_reconfigure.server import Server
# from velocities.cfg import plannerConfig


# def callback(config, level):
#     rospy.loginfo("""Reconfiugre Request: {HorVel}, {RotVel},{VerVel}, {Altitude}""".format(**config))
#     global horVel
#     horVel = HorVel.format(**config)
#     return config
global algo_ctrl
global wait
global last_precedence
algo_ctrl = False
wait = False
last_precedence = 0

def arming():
	global args
	state = True
	try:
		arming_cl = rospy.ServiceProxy(args.mavros_ns + "/cmd/arming", CommandBool)
		ret = arming_cl(value=state)
	except rospy.ServiceException as ex:
		fault(ex)

def kill():
	global args
	state = False
	try:
		arming_cl = rospy.ServiceProxy(args.mavros_ns + "/cmd/arming", CommandBool)
		ret = arming_cl(value=state)
	except rospy.ServiceException as ex:
		fault(ex)

def landing():
	global args
	try:
		land_cl = rospy.ServiceProxy(args.mavros_ns + "/cmd/land", CommandTOL)

		ret = land_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
	except rospy.ServiceException as ex:
		fault(ex)

def takeoff():
	mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
	resp = mode(0,"AUTO.TAKEOFF")
	# print "setmode send ok %s" % resp.success


    # try:
    #     takeoff_cl = rospy.ServiceProxy(args.mavros_ns + "/cmd/takeoff", CommandTOL)
        
    #     ret = takeoff_cl(altitude=2, latitude=0, longitude=0, min_pitch=0, yaw=0)
    # except rospy.ServiceException as ex:
    #     fault(ex)

def Offboard():
	resp = mode(0,"OFFBOARD")
	# print "setmode send ok %s" % resp.success

def forward(speed):
	horVel = speed
	pub.publish(twist_obj(1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0))	 

def backward(speed):
	horVel = speed
	pub.publish(twist_obj(-1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0))	 

def left(speed):
	horVel = speed
	pub.publish(twist_obj(0.0, 1*horVel, 0.0, 0.0, 0.0, 0.0))

def right(speed):
	horVel = speed
	pub.publish(twist_obj(0.0, -1*horVel, 0.0, 0.0, 0.0, 0.0))

def up(speed):
	verVel = speed
	pub.publish(twist_obj(0.0, 0.0, 1*verVel, 0.0, 0.0, 0.0))

def down(speed):
	verVel = speed
	pub.publish(twist_obj(0.0, 0.0, -1*verVel, 0.0, 0.0, 0.0))

def turn_left(speed):
	rotVel = speed
	pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, -1*rotVel))

def turn_right(speed):
	rotVel = speed
	pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, -1*rotVel))

def hover():
	pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

def custom(twisted):
	#print("custom1")
	move_cmd=TwistStamped(header=Header(stamp=rospy.get_rostime()))
	move_cmd.twist.linear.x=twisted.linear.x
	move_cmd.twist.linear.y=twisted.linear.y
	move_cmd.twist.linear.z=twisted.linear.z
	move_cmd.twist.angular.x=twisted.angular.x
	move_cmd.twist.angular.y=twisted.angular.y
	move_cmd.twist.angular.z=twisted.angular.z
	pub.publish(move_cmd)

class px4_planner:
 	def __init__(self):
 			self.sub_algo1 = rospy.Subscriber("/algo/roadfollowing", algomsg, self.cmd_call)
			#rospy.spin()
			self.orig_settings = termios.tcgetattr(sys.stdin)

			tty.setraw(sys.stdin)
	


	def cmd_call(self, data):
		algo = data.algo
		cmd = data.command
		speed = data.speed
		abs_speed = data.abs_speed
		precedence = data.precedence
		twist= data.cmd_custom

		
		precedence_good = order_check(precedence)
		if (precedence_good):
			global wait
			global algo_ctrl
			velocity = vel_def(speed,abs_speed)
			# print (velocity)
			if (algo_ctrl):
				if(wait):
					print ("waiting")
					wait = False
					time.sleep(5) 
				else:
					# print(algo)
					# print(cmd )
					# print(speed )
					# print(abs_speed)
					# print(precedence)
					if (cmd == 'forward'):
						forward(velocity)
					elif (cmd == 'backward'):
						backward(velocity)
					elif (cmd == 'left'):
						left(velocity)
					elif (cmd == 'right'):
						right(velocity)
					elif (cmd == 'up'):
						up(velocity)
					elif (cmd == 'down'):
						down(velocity)
					elif (cmd == 'turn_left'):
						turn_left(velocity)
					elif (cmd == 'turn_right'):
						turn_left(velocity)
					elif (cmd == 'takeoff'):
						takeoff()
					elif (cmd == 'kill'):
						kill()
					elif (cmd == 'arming'):
						arming()
					elif (cmd == 'land'):
						landing()
					elif(cmd == 'hover'):
						hover()
					elif(cmd == 'custom'):
						custom(twist)
						#print("custom")
					else:
						hover()



def order_check(prec):
		global last_precedence
		#print(last_precedence)
		if (last_precedence>=prec): 
			#print("new precedence")
			precedence_good = True
		else:
			precedence_good = False
		last_precedence = prec

		return precedence_good			

def vel_def(speed, abs_speed):
	if speed == 'low':
		vel = 0.5
	elif speed == 'medium':
		vel = 1.5
	elif speed == 'high':
		vel = 2.5
	elif speed == 'abs'	:
		vel = abs_speed
	else :
		vel = 0.5	
		print (" no speed selected switching to default")

	return vel


def land():
    try:
        land_cl = rospy.ServiceProxy(args.mavros_ns + "/cmd/land", CommandTOL)
        
        ret = land_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as ex:
        fault(ex)

def fault(args):
	rospy.loginfo("exception detected")
	rospy.loginfo(args)


def arm(args, state):
    try:
        arming_cl = rospy.ServiceProxy(args.mavros_ns + "/cmd/arming", CommandBool)
        ret = arming_cl(value=state)
    except rospy.ServiceException as ex:
        fault(ex)

def velocity_call():
	print("He!")
	rospy.loginfo("MAV-Teleop: Velocity setpoint control type.")

	mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
	global pub
	pub = SP.get_pub_velocity_cmd_vel(queue_size=10)
	global wait
	global algo_ctrl

	while 1:
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
		    tty.setraw(sys.stdin.fileno())
		    ch = sys.stdin.read(1)
		finally:
		    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)



		x = ch
		wait = True

		#pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

		if (x == chr(27)):
			#import tty  #required this import here
			tty.tcsetattr(fd, termios.TCSADRAIN, old_settings)
			os._exit(0)

		global horVel 
		global rotVel 
		global verVel 
		horVel = 0.5
		rotVel = 1.0
		verVel = 0.5
		print('\n'+"Keypressed-->"+x+'\n')
		
		#pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		if (x == 'a'):
			pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, -1*rotVel))
		elif (x == 'd'):
			pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 1*rotVel))
		elif (x == 'w'):
			pub.publish(twist_obj(0.0, 0.0, 1*verVel, 0.0, 0.0, 0.0))
		elif (x == 's'):
			pub.publish(twist_obj(0.0, 0.0, -1*verVel, 0.0, 0.0, 0.0))
		elif (x == 'u'):
			pub.publish(twist_obj(1*horVel, 1*horVel, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'o'):
			pub.publish(twist_obj(1*horVel, -1*horVel, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'i'):
			pub.publish(twist_obj(1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'k'):
			pub.publish(twist_obj(-1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'j'):
			pub.publish(twist_obj(0.0, 1*horVel, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'l'):
			pub.publish(twist_obj(0.0, -1*horVel, 0.0, 0.0, 0.0, 0.0))
		elif (x == '.'):
			land(args)
		elif (x == ','):
			# takeoff(args)
			resp = mode(0,"AUTO.TAKEOFF")
			pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'm'):
			arm(args,True)
			resp = mode(0,"OFFBOARD")
			# print "setmode send ok %s" % resp.success
		elif (x == '/'):
			arm(args, False)
		elif (x == 'z'):
			algo_ctrl = not algo_ctrl
			print(algo_ctrl)
			wait = False
			# px4_control()

		else:
			pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

	# rospy.spin()

	
#threading.Thread(target = velocity_call).start()


def twist_obj(x,y,z,a,b,c):
	move_cmd=TwistStamped(header=Header(stamp=rospy.get_rostime()))
	move_cmd.twist.linear.x=x
	move_cmd.twist.linear.y=y
	move_cmd.twist.linear.z=z
	move_cmd.twist.angular.x=a
	move_cmd.twist.angular.y=b
	move_cmd.twist.angular.z=c
	return move_cmd

def main():
	# Initialize Node class
	parser = argparse.ArgumentParser(description="Teleoperation script for Copter-UAV")
	rospy.init_node('px4_control', anonymous=True)
	parser.add_argument('-n', '--mavros-ns', help="ROS node namespace", default="/mavros")
	global args
	args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
	threading.Thread(target = velocity_call).start()
	mavros.set_namespace(args.mavros_ns)
	
	#orig_settings = termios.tcgetattr(sys.stdin)
	#tty.setraw(sys.stdin)


	px4C = px4_planner()
	# spin the node
	try:
	 	rospy.spin()
	 # # stop the node if Ctrl-C pressed
	except KeyboardInterrupt:
	 	print("Shutting down")
	 	cv2.destroyAllWindows()

if __name__ == '__main__':
	print("Hello world!")
	# global algo
	# global wait
	# algo = False
	# wait = False
	# srv = Server(plannerConfig,callback)	
main()