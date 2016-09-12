#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy



def callback(data):
<<<<<<< HEAD
=======
    twist = Twist()
>>>>>>> ab122e0f02055efea28a1ec1254835c1c6eba6d2
    twist.linear.x = 1*data.axes[1]
    twist.angular.z = 4*data.axes[0]
    pub.publish(twist)
    
def twistStatus_callback(data):
	pub.publish(twist)


def start():
    global pub
    global pub_2
    
    rospy.init_node('Joy_robot')
<<<<<<< HEAD
    
=======
    global pub
>>>>>>> ab122e0f02055efea28a1ec1254835c1c6eba6d2
    rospy.Subscriber("~input", Joy, callback)
	rospy.Subscriber("/vrep/twistStatus", Twist, twistStatus_callback)
	
    pub = rospy.Publisher('~output', Twist)
<<<<<<< HEAD
    pub_2 = rospy.Publisher("/vrep/twistStatus_2", Twist)
    
    """
    rate = rospy.Rate(34) # 60 hz or 34 to match publish rate of /vrep/twistStatus
    while not rospy.is_shutdown():
		pub.publish(twist)
		rate.sleep()
    """
=======
    rate = rospy.Rate(60)
    
>>>>>>> ab122e0f02055efea28a1ec1254835c1c6eba6d2
    rospy.spin()

if __name__ == '__main__':
    start()
