#!/usr/bin/env python3
import rospy
import roslib
roslib.load_manifest('ebobot')
#from sys import argv
rospy.init_node('pid_setter')
path = rospy.get_param('pid_setter/path', 'config/scripts/pid_settings.txt') 
from std_msgs.msg import Float32
rospy.sleep(5)
rospy.loginfo(f"Setting PID from {path}")
pid_msg = Float32()
pub = rospy.Publisher('set_pid', Float32 ,queue_size = 15)
rospy.sleep(1)
with open(path, 'r') as f: 
    pids = f.readlines()
    for coeff in pids:
        for val in coeff.split(" "):
            pid_msg.data = float(val) 
            pub.publish(pid_msg)
            rospy.loginfo(f"Published {val}")
            rospy.sleep(0.1)


