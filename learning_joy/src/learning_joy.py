#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

twist = Twist()

def callback(data):
    twist.linear.x = 1*data.axes[1]
    #twist.angular.z = 4*data.axes[0]
    #pub.publish(twist)
    
def twistStatus_callback(data):
	pub.publish(twist)


def start():
    global pub
    global pub_2
    
    rospy.init_node('Joy_robot')
    
    rospy.Subscriber("~input", Joy, callback)
	rospy.Subscriber("/vrep/twistStatus", Twist, twistStatus_callback)
	
    pub = rospy.Publisher('~output', Twist)
    pub_2 = rospy.Publisher("/vrep/twistStatus_2", Twist)
    
    """
    rate = rospy.Rate(34) # 60 hz or 34 to match publish rate of /vrep/twistStatus
    while not rospy.is_shutdown():
		pub.publish(twist)
		rate.sleep()
    """
    rospy.spin()

if __name__ == '__main__':
    start()
