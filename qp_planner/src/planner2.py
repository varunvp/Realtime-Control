#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
#left_yaw is positive angular z
#right_yaw is negative angular z
from __future__ import print_function

import os, sys, select, termios, tty
import roslib, rospy
import argparse
import rospy
import mavros
import threading, time

from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist
from subprocess import call
from mavros_msgs.msg import OverrideRCIn
from mavros import command
from mavros import setpoint as SP
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import PositionTarget

from qp_planner.msg import algomsg
from mavros_msgs.msg import Altitude


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
global altitude
algo_ctrl = False
wait = False
count = 0
last_precedence = 0
altitude = 0.000000

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
	pub.publish(twist_obj(0.0, 1*horVel, 0.0, 0.0, 0.0, 0.0))	 

def backward(speed):
	horVel = speed
	pub.publish(twist_obj(0.0,-1*horVel, 0.0, 0.0, 0.0, 0.0))	 

def left(speed):
	horVel = speed
	pub.publish(twist_obj(-1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0))

def right(speed):
	horVel = speed
	pub.publish(twist_obj(1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0))

def up(speed):
	verVel = speed
	pub.publish(twist_obj(0.0, 0.0, 1*verVel, 0.0, 0.0, 0.0))

def down(speed):
	verVel = speed
	pub.publish(twist_obj(0.0, 0.0, -1*verVel, 0.0, 0.0, 0.0))

def turn_left(speed):
	rotVel = speed
	pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 1*rotVel))
	#print ("LEFT me hai !!")-1*
	#print(rotVel)

def turn_right(speed):
	rotVel = speed
	pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, -1*rotVel))
	#nt(rotVel)

def hover():
	pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

def custom(twisted):
	#print("custom1")
	move_cmd=PositionTarget(header=Header(stamp=rospy.get_rostime()))
	move_cmd.velocity.x=twisted.linear.y
	move_cmd.velocity.y=twisted.linear.x
	move_cmd.velocity.z=twisted.linear.z
	move_cmd.yaw_rate=twisted.angular.z
	move_cmd.coordinate_frame=8
	move_cmd.type_mask=1479 # 1479/ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0)
	pub.publish(move_cmd)

class px4_planner:
 	def __init__(self):
 			self.sub_algo1 = rospy.Subscriber("/algo/roadfollowing", algomsg, self.cmd_call)
			#rospy.spin()
			self.orig_settings = termios.tcgetattr(sys.stdin)
			self.alt_sub = rospy.Subscriber("/mavros/altitude", Altitude, self.alt_callback)


			tty.setraw(sys.stdin)
	
	def alt_callback(self, data):
		#print("$$$$$$$$$Altitude$$$$$$$$$$$")
                print(data.bottom_clearance)
		global altitude
                altitude = data.bottom_clearance


	def cmd_call(self, data):
		algo = data.algo
		cmd = data.command
		speed = data.speed
		abs_speed = data.abs_speed
		precedence = data.precedence
		twist= data.cmd_custom
		#print("Hello cmd")

		
		precedence_good = order_check(precedence)
		if (precedence_good):
			if precedence ==1:
				print("road-following/landing\n")
			elif precedence == 2:
				print("UWB=turning")
			global wait
			global algo_ctrl
			velocity = vel_def(speed,abs_speed)
			#print (velocity)
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
						turn_right(velocity)
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
					elif(cmd == 'disarm'):
						arm(args, False)
					elif(cmd == 'custom'):
						custom(twist)
						#print("custom")
					else:
						hover()



def order_check(prec):
		global last_precedence,count
		#print(last_precedence)
		#print(prec)
		if (last_precedence<=prec): 
			#print("good precedence")
			precedence_good = True
			last_precedence = prec
			count = 0
		else:
			count = count +1
			precedence_good = False
			#print("low_prec")
			if count > 25:
				last_precedence = 0
				count = 0
				print("prec_reset")
		return precedence_good			

def vel_def(speed, abs_speed):
	if speed == 'low':
		vel = 0.8
	elif speed == 'medium':
		vel = 1.2
	elif speed == 'high':
		vel = 2.5
	elif speed == 'abs'	:
		vel = abs_speed
	else :
		vel = 0.6	
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
	pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
	#pub = rospy.Publisher('/mavros/setpoint_raw/target_local', PositionTarget, queue_size=10)
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
		global altitude 
		horVel = 2.0
		rotVel = 1.0
		verVel = 2.0
		print('\n'+"Keypressed-->"+x+'\n')
		
		#pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		if (x == 'a'):
			pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 1*rotVel))
		elif (x == 'd'):
			pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, -1*rotVel))
		elif (x == 'w'):
			pub.publish(twist_obj(0.0, 0.0, 1*verVel, 0.0, 0.0, 0.0))
		elif (x == 's'):
			pub.publish(twist_obj(0.0, 0.0, -1*verVel, 0.0, 0.0, 0.0))
		elif (x == 'u'):
			pub.publish(twist_obj(1*horVel, 1*horVel, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'o'):
			pub.publish(twist_obj(1*horVel, -1*horVel, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'l'):
			pub.publish(twist_obj(1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'j'):
			pub.publish(twist_obj(-1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'i'):
			pub.publish(twist_obj(0.0, 1*horVel, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'k'):
			pub.publish(twist_obj(0.0, -1*horVel, 0.0, 0.0, 0.0, 0.0))
		elif (x == '.'):
			land()
		elif (x == ','):
			# takeoff(args)
			resp = mode(0,"AUTO.TAKEOFF")
			pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		elif (x == 'm'):
			arm(args,True)
			resp = mode(0,"OFFBOARD")
			#print "setmode send ok %s" % resp.success
		elif (x == '/'):
			arm(args, False)
		elif (x == 'z'):
			algo_ctrl = not algo_ctrl
			print(algo_ctrl)
			wait = False
			# px4_control()
		elif (x == 't'):
			r = rospy.Rate(10)
			while (altitude<1.5):
				pub.publish(takeoff_obj(2.5))
				r.sleep()
			while (altitude>1.4):
				algo_ctrl = not algo_ctrl
				#loiter()
				r.sleep()
				print("-----------altitude reached loiter----------")
		else:
			pub.publish(twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

	# rospy.spin()

	
#threading.Thread(target = velocity_call).start()


def twist_obj(x,y,z,a,b,c):
	move_cmd=PositionTarget(header=Header(stamp=rospy.get_rostime()))
	move_cmd.velocity.x=x
	move_cmd.velocity.y=y
	move_cmd.velocity.z=z
	move_cmd.yaw_rate=c
	move_cmd.coordinate_frame=8
	move_cmd.type_mask=1479 # 1479/ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0)
	return move_cmd

def takeoff_obj(z):
	print("--------------------Takeoff-----------------")
	move_cmd=PositionTarget(header=Header(stamp=rospy.get_rostime()))
	move_cmd.velocity.x=0
	move_cmd.velocity.y=0
	move_cmd.velocity.z=0.5
	move_cmd.position.z=z
	move_cmd.type_mask=(0x1000|0b110111000011)
	move_cmd.coordinate_frame=9
	return move_cmd

def loiter():
	print("--------------------Takeoff-----------------")
	move_cmd=PositionTarget(header=Header(stamp=rospy.get_rostime()))
	move_cmd.velocity.x=0
	move_cmd.velocity.y=0
	move_cmd.velocity.z=0
	move_cmd.position.x=0
	move_cmd.position.y=0
	move_cmd.position.z=0
	move_cmd.type_mask=(0x2000|0b110111000011)
	move_cmd.coordinate_frame=9
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
