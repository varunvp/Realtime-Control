#!/usr/bin/python

import rospy, tty, sys, select, termios
from std_msgs.msg import Bool

settings = termios.tcgetattr(sys.stdin)

def isData():
    return select.select([sys.stdin], [], [], 0.05) == ([sys.stdin], [], [])

def getKey():
	tty.setraw(sys.stdin.fileno())
	# select.select([sys.stdin], [], [], 0.1)
	key = None
	if isData():
		key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

rospy.init_node("dead_man_switch")
switch_pub = rospy.Publisher("kill_publish", Bool, queue_size=1)

while not rospy.is_shutdown():
	key = getKey()
	if(key):
		switch_pub.publish(True)
		print("Publishing")

	else:
		switch_pub.publish(False)