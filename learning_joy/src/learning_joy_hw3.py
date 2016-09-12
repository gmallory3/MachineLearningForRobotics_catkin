#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

twistCommand = Twist()
twistStatus = Twist()

def callback_command(data):
    twistCommand.linear.x = 1*data.axes[1]
    #twist.angular.z = 4*data.axes[0]
    pub_command.publish(twistCommand)
    rospy.loginfo('nik')
    
def callback_status(data):
    twistStatus.linear.x = data.linear.x
    rospy.loginfo('nik')
    pub_status.publish(twistStatus)
    
def start():
    rospy.init_node('Joy_robot')
    global pub_command 
    pub_command = rospy.Publisher("~output", Twist)
    rospy.Subscriber("~input", Joy, callback_command)
    global pub_status 
    pub_status = rospy.Publisher("vrep/twistStatusSync/", Twist)
    rospy.Subscriber("vrep/twistStatus/", Twist, callback_status)
    
    #rate = rospy.Rate(60)

    rospy.spin()

if __name__ == '__main__':
    start()
