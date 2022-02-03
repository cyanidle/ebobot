#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
##############
coeffs = [[210,210,210],    #prop_coeff
         [220,220,220],    #inter_coeff
         [10,10,10]]       #diff_coeff


#############
rospy.init_node('pid_setter_node')
pub = rospy.Publisher('set_pid', Float32 ,queue_size = 15)
rospy.sleep(1)
for coeff in coeffs:
    for val in coeff: 
        pub.publish(Float32(val))
        rospy.loginfo(f"Published {val}")
        rospy.sleep(0.1)


