#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
##############
coeffs = [[120,120,120],    #prop_coeff
         [150,150,150],    #inter_coeff
         [10,10,10]]       #diff_coeff


#############
rospy.init_node('pid_setter_node')
pub = rospy.Publisher('set_pid', Float32 ,queue_size = 15)
for coeff in coeffs:
    for val in coeff:
        rospy.loginfo(f"Published {val}")
        pub.publish(Float32(val))
        rospy.sleep(0.1)

