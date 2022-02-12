import roslib
roslib.load_manifest('ebobot')
import rospy
import cmath
import tf
rospy.init_node('local_planer')
from libraries.Dorvect import Dorvect
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
        target = [pose.pose.position.x,pose.pose.position.y,tf.transformations.quaternion_from_euler(.pose.orientation)]
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
    current_target_pos = Dorvect([0,0,0])
    #/Global values

    #def __init__(self):
    def recalcCostCoordsFromRadius(radius):
        Local.x_list.clear()
        Local.cost_coords_list.clear()
        Local.precalcCostCoordsFromRadius(radius)
    def precalcCostCoordsFromRadius(radius):
        rad = radius                           #Local.safe_footprint_radius
        vect = Dorvect(rad,0,0)
        x_list = []
        x_list.append((0,rad))
        last_x = 0
        for i in range(cmath.pi//(Local.step_radians*2)+1): #This shit here casts some vectors with dist = rad in a quarter circle #Should fix for different rad steps.  
            vect.updateFromImag(vect.imag * (cmath.cos(-Local.step_radians) + 1j * cmath.sin(-Local.step_radians))) #This may cause a bug later (excess coords) or (missing coords) near y=0, x = rad
            for x in range(last_x,round(vect.x)+1):
                if Local.debug:
                    rospy.loginfo(f"Precalc max x --> {x}, max y --> {round(vect.y)}")
                x_list.append((x, round(vect.y)))
            last_x = round(vect.x)
        for x_max, y_max in x_list:
            for x in range(-x_max,x_max+1):
                for y in range(-y_max,y_max+1):
                    Local.cost_coords_list.append((x,y))
    def reset():
        Local.current_target = -1 

    def getCost(curr_x, curr_y):
        cost = 0
        for x,y in Local.cost_coords_list:
            cost += Local.costmap[[curr_x+x],[curr_y+y]]
        return cost
    #def getPoses():
    def cmdVel():
        twist = Twist()
        targ_vect = getattr(Local.robot_pos - Local.current_target_pos,"vect")  
        move = targ.vect/np.linalg.norm(targ_vect)*Local.cost_speed_coeff*Local.getCost(targ_vect[0],targ_vect[1]) #make param
        twist.linear.x = move[0]
        twist.angular.y = move[1]
        twist.angular.z = move[2] #make slower at last point
        cmd_vel_publisher.publish(twist)
    def updateTarget():
        if abs(Local.robot_pos-Local.current_target_pos) < Local.threshhold: #make param
if __name__ == "__main__":
    #Topics
    path_subscriber = rospy.Subscriber(Local.path_subscribe_topic, Path, pathCallback)
    cmd_vel_publisher = rospy.Publisher(Local.cmd_vel_topic, Twist, queue_size = 25)
    robot_pos_subscriber = rospy.Subscriber(Local.robot_pos_topic, PoseStamped, robotPosCallback)
    #/Topics
    #epic
