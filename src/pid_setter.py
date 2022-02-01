#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
##############
coeffs = [400,400,400],    #prop_coeff
         [500,500,500],    #inter_coeff
         [30,30,30]]       #diff_coeff


#############
rospy.init_node('pid_setter_node')
pub = rospy.Publisher('set_pid', Float32)
for coeff in coeffs:
    for val in coeff:
        pub.publish(Float32(val))
        rospy.sleep(0.1)

