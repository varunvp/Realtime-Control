#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Header,String
from px4_planner.msg import algomsg
from geometry_msgs.msg import Twist

def algomsged(algo, command, speed, abs_speed, precedence,x,y,z,a,b,c):
  cmd = algomsg(header=Header(stamp=rospy.get_rostime()))
  cmd.algo = algo
  cmd.command = command
  cmd.speed = speed
  cmd.abs_speed = abs_speed
  cmd.precedence = precedence
  cmd.cmd = twist_obj(x,y,z,a,b,c)
  return cmd

def compact_algomsg(command, speed):
 cmd=algomsged("Road-Following", command, speed, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
 return cmd

def twist_obj(x,y,z,a,b,c):
  move_cmd=Twist()
  move_cmd.linear.x=x
  move_cmd.linear.y=y
  move_cmd.linear.z=z
  move_cmd.angular.x=a
  move_cmd.angular.y=b
  move_cmd.angular.z=c
  return move_cmd

def talker():
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    pub = rospy.Publisher('/algo/roadfollowing', algomsg, queue_size=3)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        #pub.publish(algomsged("Road_following", "forward", "high", 0, 1))
        pub.publish(compact_algomsg("forward", "high"))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass