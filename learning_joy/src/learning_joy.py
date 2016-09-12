#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def callback(data):
    twist = Twist()
    twist.linear.x = 1*data.axes[1]
    twist.angular.z = 4*data.axes[0]
    pub.publish(twist)
    
def twistStatus_callback(data):
	pub.publish(twist)


def start():    
    rospy.init_node('Joy_robot')
    global pub
    rospy.Subscriber("~input", Joy, callback)	
    pub = rospy.Publisher('~output', Twist)
    rate = rospy.Rate(60)
    rospy.spin()

if __name__ == '__main__':
    start()
