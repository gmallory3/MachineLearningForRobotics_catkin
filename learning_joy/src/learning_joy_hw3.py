#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

twistCommand = Twist()

def callback_command(data):
	global twistCommand
	twist = Twist()
	twist.linear.x = 1*data.axes[1]
	twist.angular.z = 1*data.axes[3]
	twistCommand = twist

def callback_status(data):
	global twistCommand
	pub.publish(twistCommand)
	
def start():
	global pub
	
	rospy.init_node('Joy_robot')
	
	pub = rospy.Publisher("~output", Twist)
	rospy.Subscriber("~input_0", Joy, callback_command)
	rospy.Subscriber("~input_1", TwistStamped, callback_status)
	rospy.spin()
if __name__ == '__main__':
    start()
