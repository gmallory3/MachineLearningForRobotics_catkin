#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

twist = Twist()

def callback(data):
    twist.linear.x = 1*data.axes[1]
    twist.angular.z = 1*data.axes[0]
    pub.publish(twist)
    

def start():    
    rospy.init_node('Joy_robot')
    global pub
    rospy.Subscriber("~input", Joy, callback)	
    pub = rospy.Publisher('~output', Twist)

    rospy.spin()

if __name__ == '__main__':
    start()
