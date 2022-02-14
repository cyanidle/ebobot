import roslib
roslib.load_manifest('ebobot')
import rospy
import cmath
import tf
rospy.init_node('local_planer')
from libraries.DorLib import Dorvect, deltaCoordsInRad
#The planer should try planing the path using radius of the robot and avoiding obstacles, but if the next point is unreachable, skip it and reroute
#To the next
#Messages and actions
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Path, OccupancyGrid, Odometry
######

def robotPosCallback(pose):
    Local.robot_pos = Dorvect([pose.pose.x,pose.pose.y,tf.transformations.euler_from_quarternion(pose.pose.orientation)[2]])
def pathCallback(path):################Доделать
    for pose in path:
        target = [pose.pose.position.x,pose.pose.position.y,tf.transformations.quaternion_from_euler(pose.pose.orientation)]
        Local.targets.append(target)
    Local.reset()
#Field :   204x304 cm
class Local():
    #Params
    robot_pos_topic = rospy.get_param('local_planer/robot_pos_topic', '/odom')
    cmd_vel_topic = rospy.get_param('local_planer/cmd_vel_topic', '/cmd_vel')
    debug = rospy.get_param('local_planer/debug', 1)
    path_subscribe_topic =  rospy.get_param('local_planer/path_subscribe_topic', '/global_path') #### TAKEN FROM GLOBAL_PLANER PARAMS
    num_of_circles = rospy.get_param('local_planer/num_of_circles', 3)
    step_radians = rospy.get_param('local_planer/step_radians', cmath.pi/4)
    #### Params for footprint cost calc
    footprint_calc_step_radians = rospy.get_param('local_planer/footprint_calc_step_radians', 0.3)
    #calculate_base_cost = rospy.get_param('local_planer/calculate_base_cost', 1)
    base_footprint_radius = rospy.get_param('local_planer/base_footprint_radius', 0.20) #optional
    safe_footprint_radius = rospy.get_param('local_planer/safe_footprint_radius', 0.25)
    #### /Params for footprint cost calc
    circles_dist = rospy.get_param('local_planer/circles_dist', base_footprint_radius/num_of_circles)
    #/Params

    #Global values
    robot_pos = Dorvect([0,0,0])
    costmap = []
    cost_coords_list = []
    targets = []
    current_target = 0
    #/Global values

    #def __init__(self):
    def recalcCostCoordsFromRadius(radius):
        Local.x_list.clear()
        Local.cost_coords_list.clear()
        Local.precalcCostCoordsFromRadius(radius)
    def precalcCostCoordsFromRadius():
        Local.cost_coords_list = [(x,y) for x,y in deltaCoordsInRad(Local.safe_footprint_radius,Local.step_radians)]
    def reset():
        Local.current_target = 0
    def getCost():
        cost = 0
        for x,y in Local.cost_coords_list:
            cost += Local.costmap[[x],[y]]
        return cost
    #def getPoses():
    def cmdVel():
        twist = Twist()
        #twist.linear.x = 
        #twist.angular.y =
        #twist.angular.z =
        cmd_vel_publisher.publish(twist)

if __name__ == "__main__":
    #Topics
    path_subscriber = rospy.Subscriber(Local.path_subscribe_topic, Path, pathCallback)
    cmd_vel_publisher = rospy.Publisher(Local.cmd_vel_topic, Twist, queue_size = 25)
    robot_pos_subscriber = rospy.Subscriber(Local.robot_pos_topic, PoseStamped, robotPosCallback)
    #/Topics
    #epic