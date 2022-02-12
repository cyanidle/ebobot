import roslib
roslib.load_manifest('ebobot')
import rospy
import cmath
import tf
rospy.init_node('local_planer')
from libraries.Dorvect import Dorvect
#The planer should try planing the path using radius of the robot and avoiding obstacles, but if the next point is unreachable, skip it and reroute
#To the next

class Local():
    #Params
    path_subscriber_topic =  rospy.get_param('global_planer/path_publisher_topic', 'global_path') #### TAKEN FROM GLOBAL_PLANER PARAMS
    num_of_circles = rospy.get_param('local_planer/path_publisher_topic', 3)
    step_radians = rospy.get_param('local_planer/step_radians', cmath.pi/4)
    base_footprint_radius = rospy.get_param('local_planer/circles_dist', 0.20)
    circles_dist = rospy.get_param('local_planer/circles_dist', base_footprint_radius/num_of_circles)
    #/Params
    def __init__(self):
    def precalculateFromRadius():
    def getCost(self):
    def getPoses(self):
    def cmdVel(self):
