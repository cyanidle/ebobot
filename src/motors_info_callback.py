#!/usr/bin/env python3
import roslib; 
import rospy
from std_msgs.msg import Float32MultiArray
info_len = 12
num_motors = 3

def callback(info):
    motors_currs = {info[0],info[4],info[8]}
    motors_targs =  {info[1],info[5],info[9]}
    motors_dists = {info[2],info[6],info[10]}
    motors_ddists = {info[3],info[7],info[11]}

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("motors_info", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener() 

