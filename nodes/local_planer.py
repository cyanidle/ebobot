#!/usr/bin/env python3
import roslib
import rospy
import cmath
import tf
import numpy as np
#The planer should try planing the path using radius of the robot and avoiding obstacles, but if the next point is unreachable, skip it and reroute
#To the next
#Messages and actions
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
######
from dorlib import dCoordsInRad,dCoordsOnCircle
######Callbacks
def robotPosCallback(pose):
    quat = [pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w]
    Local.robot_pos = np.array([pose.pose.pose.x,pose.pose.pose.y,tf.transformations.euler_from_quarternion(quat)[2]]) / Local.costmap_resolution
def pathCallback(path):################Доделать
    Local.clearTargets()
    for pose in path:
        quat = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        target = np.array([pose.pose.position.x,pose.pose.position.y,tf.transformations.euler_from_quarternion(quat)[2]])
        Local.new_targets.append(target)
    Local.parseTargets()
    Local.goal_reached = 0
def costmapCallback(costmap):
    rospy.loginfo("Got new map")
    Local.costmap_resolution = costmap.info.resolution
    Local.costmap_width = costmap.info.width
    Local.costmap_height = costmap.info.height
    for y in range(costmap.info.width+1):
        for x in range(costmap.info.height+1):
            Local.costmap[x][y] = costmap.data[x+y]
    pass #Dodelai
def costmapUpdateCallback(update):
    origin_x = update.x
    origin_y = update.y
    for x in range (update.height+1):
        for y in range (update.width+1):
            Local.costmap[origin_x + x][origin_y + y] = update.data[x+y]
######/Callbacks
#Field :   204x304 cm
class Local():
    rospy.init_node('local_planer')
    roslib.load_manifest('ebobot')
    #Params
    min_speed_coeff = rospy.get_param('local_planer/min_speed_coeff', 0.3)
    path_speed_coeff = rospy.get_param('local_planer/path_speed_coeff', 1)
    cost_threshhold = rospy.get_param('local_planer/cost_threshhold', 400) #100 are walls, then there is inflation
    num_of_steps_between_globals = rospy.get_param('local_planer/num_of_steps_between_globals', 4)
    update_rate = rospy.get_param('local_planer/update_rate', 10) # in Hz
    cost_speed_coeff = rospy.get_param('local_planer/cost_speed_coeff', 0.2)
    threshhold = rospy.get_param('local_planer/threshhold', 2) #in cells
    costmap_topic = rospy.get_param('local_planer/costmap_topic', '/costmap')
    costmap_update_topic = rospy.get_param('local_planer/costmap_update_topic', '/costmap_update')
    robot_pos_topic = rospy.get_param('local_planer/robot_pos_topic', '/odom')
    cmd_vel_topic = rospy.get_param('local_planer/cmd_vel_topic', '/cmd_vel')
    debug = rospy.get_param('local_planer/debug', 1)
    path_subscribe_topic =  rospy.get_param('local_planer/path_subscribe_topic', '/global_path')
    num_of_circles = rospy.get_param('local_planer/num_of_circles', 3)
    circles_dist = rospy.get_param('local_planer/circles_dist', 5) #in cells
    circles_step_radians_resolution = rospy.get_param('local_planer/circles_step_radians_resolution', 3) #number of vectors in each quarter of a circle (more = more round field)
    #### Params for footprint cost calc
    footprint_calc_step_radians_resolution = rospy.get_param('local_planer/footprint_calc_step_radians', 6) #number of vectors for footprint
    #calculate_base_cost = rospy.get_param('local_planer/calculate_base_cost', 1)
    base_footprint_radius = rospy.get_param('local_planer/base_footprint_radius', 0.20) #optional
    safe_footprint_radius = rospy.get_param('local_planer/safe_footprint_radius', 0.30)
    #### /Params for footprint cost calc
    #/Params

    #Topics
    costmap_update_subscriber = rospy.Subscriber(costmap_update_topic, OccupancyGridUpdate, costmapUpdateCallback)
    costmap_subscriber = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback)
    path_subscriber = rospy.Subscriber(path_subscribe_topic, Path, pathCallback)
    cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 25)
    robot_pos_subscriber = rospy.Subscriber(robot_pos_topic, Odometry, robotPosCallback)
    #/Topics

    #global values
    skipped = 0
    goal_reached = 1
    new_targets = []
    costmap_resolution = 0.02
    robot_pos = np.array([0,0,0])
    default_costmap_list = [[0]*101 for _ in range(151)]
    costmap = np.array(default_costmap_list)
    costmap_height = 151
    costmap_width = 101
    cost_coords_list = dCoordsInRad(safe_footprint_radius,footprint_calc_step_radians_resolution)
    targets = []
    current_target = 0
    cost_check_poses = []
    for n in range(num_of_circles):
        for x,y in dCoordsOnCircle(n*circles_dist,circles_step_radians_resolution):
            cost_check_poses.append((x,y))
    #/global values
    ##############################deltaCoordsPrecalc
    @staticmethod
    def recalcCostCoordsFromRadius(radius):
        Local.cost_coords_list = [(x,y) for x,y in dCoordsInRad(radius/Local.costmap_resolution,Local.footprint_calc_step_radians_resolution)]
    #############################/deltaCoordsPrecalc
    @staticmethod
    def clearTargets():
        Local.new_targets.clear()
        Local.targets.clear()
        Local.current_target = 0 
    @staticmethod
    def parseTargets():
        current_theta = Local.robot_pos[2]
        final_target = Local.new_targets.pop()
        delta_theta = (final_target[2] - current_theta) / (len(Local.targets) + 1)
        new_parsed_targets = []
        for num,target in enumerate(Local.targets):
            new_parsed_targets.append(target[:2].append(delta_theta * num))
        new_parsed_targets.append(final_target)
        Local.targets = new_parsed_targets
        if Local.debug:
            rospy.loginfo(f"Parsed targets = {Local.targets}")
    @staticmethod
    def getCost(pose):
        curr_x,curr_y = pose[0], pose[1]
        cost = 0
        for x,y in Local.cost_coords_list:
            cost += Local.costmap[[curr_x+x],[curr_y+y]]
        return cost
    
    @staticmethod
    def cmdVel(target,speed_coeff):       #speed ranges from 0 to 1
        twist = Twist()
        move =  target/np.linalg.norm(target)*speed_coeff#make param
        twist.linear.x = move[0]
        twist.angular.y = move[1]
        twist.angular.z = move[2] #make slower at last point
        Local.cmd_vel_publisher.publish(twist)



    @staticmethod
    def getPathSpdCoeff():
        max_targets = len(Local.targets)
        coeff = abs(Local.current_target - max_targets/2) / (max_targets/2)
        final_coeff = coeff * Local.path_speed_coeff
        if final_coeff < Local.min_speed_coeff:
            final_coeff = Local.min_speed_coeff
        rospy.loginfo(f"Fetched speed coeff from dist to goal = {final_coeff}")
        return final_coeff
    @staticmethod
    def fetchPoint(current, target):
        if Local.debug:
            rospy.loginfo(f"Fetching point with curr = {current}, targ = {target}")
        for num in range(Local.num_of_steps_between_globals*Local.skipped):
            curr_targ = (Local.targets[target] - current) /num
            min_cost = 0
            for x,y in Local.cost_check_poses:
                pose = np.array([curr_targ[0] + x, curr_targ[1] + y, curr_targ[2]])
                curr_cost = Local.getCost(pose)
                if curr_cost < min_cost:
                   point = pose
                   point_cost = curr_cost
                   min_cost = curr_cost 
            if point_cost > Local.cost_threshhold:
                Local.skipped += 1
                Local.current_target += 1
                yield Local.fetchPoint(current, target+1) #or break??
            elif np.linalg.norm(curr_targ - target) < Local.threshhold:
                Local.skipped = 0
                Local.current_target += 1
                yield point
            else:
                yield point
                
    @staticmethod
    def updateTarget():
        Local.current_target +=1
        current_pos = Local.robot_pos
        target = current_pos  - Local.targets[Local.current_target]
        actual_target = Local.fetchPoint(current_pos, target)
        rospy.loginfo(f'Riding to {actual_target}')
        while np.linalg.norm(Local.robot_pos - actual_target) > Local.threshhold:
            cost_speed_coeff = Local.cost_speed_coeff*Local.getCost(target[0],target[1])
            Local.cmdVel(target, cost_speed_coeff * Local.getPathSpdCoeff())
            rospy.sleep(1/Local.update_rate)
 


def main():
    rate = rospy.Rate(Local.update_rate)
    #Local.precalcCostCoordsFromRadius()
    if Local.debug:
        rospy.loginfo(f"Cost coords = {Local.cost_coords_list}")
    while not rospy.is_shutdown():
        if not Local.goal_reached:
            current_target = Local.targets[Local.current_target]
                #if current_target == len(Local.targets):
            if np.linalg.norm(Local.robot_pos - Local.targets[-1]) < Local.threshhold:
                rospy.loginfo(f'Goal reached!')
                Local.goal_reached = 1
            if np.linalg.norm(Local.robot_pos-current_target) > Local.threshhold: #make param
                Local.updateTarget(rate)
    rate.sleep()

if __name__=="__main__":
    main()