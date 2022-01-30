#!/usr/bin/env python3
import roslib; 
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

def callback(info):
    for mot in range(0,3):
        rospy.loginfo(f" MOTOR #{mot}: target speed {info.data[mot*4]}    current speed {info.data[(mot*4)+1]}")


def listener():
    rospy.init_node('motors_info_callback', anonymous=True)
    rospy.Subscriber("motors_info", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener() 
