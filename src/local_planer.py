import roslib
roslib.load_manifest('ebobot')
import rospy
import math
import tf
rospy.init_node('local_planer')

#The planer should try planing the path using radius of the robot and avoiding obstacles, but if the next point is unreachable, skip it and reroute
#To the next
