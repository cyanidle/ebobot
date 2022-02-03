#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
rospy.init_node('pid_setter_node')
pub = rospy.Publisher('set_pid', Float32 ,queue_size = 15)
rospy.sleep(1)
with open('pid_settings.txt', 'r') as f:
    pids = f.readlines()
for coeff in pids:
    for val in coeff.split(" "): 
        pub.publish(Float32(val))
        rospy.loginfo(f"Published {val}")
        rospy.sleep(0.1)
pids.close()

