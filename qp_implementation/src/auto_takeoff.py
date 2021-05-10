#!/usr/bin/python

import rospy, npyscreen, termios, tty, mavros, sys, time
from mavros import command
from mavros_msgs.msg import Altitude, State, PositionTarget, HomePosition
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

UAV_state = State()

def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided

def main():
	rospy.init_node("uav_automation")
	rate = rospy.Rate(10)

	mavros.set_namespace('/mavros')

	state_sub = rospy.Subscriber(mavros.get_topic('state'), State, _state_callback)
	# alt_sub = rospy.Subscriber(mavros.get_topic())

	vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, queue_size = 3) 
	# setup service
	# /mavros/cmd/arming
	set_arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
	# /mavros/set_mode
	set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

	setpoint_msg = TwistStamped(
	    header=Header(
	    stamp=rospy.Time.now()),
	)
	
	# initialize the setpoint
	setpoint_msg.twist.linear.x = 0
	setpoint_msg.twist.linear.y = 0
	setpoint_msg.twist.linear.z = 0.1
	mavros.command.arming(True)

	# send 50 setpoints before starting
	for i in range(0, 50):
	    vel_pub.publish(setpoint_msg)
	    rate.sleep()
    
	set_mode(0, 'AUTO.TAKEOFF')

	last_request = rospy.Time.now()
	time.sleep(1)

	while(UAV_state.mode == "AUTO.TAKEOFF"):
		# if (UAV_state.mode != "AUTO.LOITER" and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
	   set_mode(0, 'AUTO.LOITER')
	   print("enabling hold mode")
	   last_request = rospy.Time.now()
	   if(UAV_state.mode == "POSCTL"):
	   	sys.exit()
	
	last_hold_request = rospy.Time.now()
	
	while not rospy.is_shutdown():
		if(UAV_state.mode == "AUTO.LOITER" and (rospy.Time.now() - last_hold_request > rospy.Duration(10.0))):
			print("Landing")
			set_mode(0, 'AUTO.LAND')

		print(UAV_state.mode)
		# print(rospy.Time.now() - last_hold_request)
		
		if(UAV_state.mode == "POSCTL"):
			sys.exit()


		# else:
		#     if (not UAV_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
		#         if (mavros.command.arming(True)):
		#             print("Vehicle armed")
		#         last_request = rospy.Time.now()		

		
		vel_pub.publish(setpoint_msg)
		rate.sleep()

if __name__ == '__main__':
	main()