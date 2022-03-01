#!/usr/bin/env python3
import roslib
import rospy
from math import sin, cos
import tf
import numpy as np
#The planer should try planing the path using radius of the robot and avoiding obstacles, but if the next point is unreachable, skip it and reroute
#To the next
#Messages and actions
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import Marker
######
from dorlib import turnVect, dCoordsOnCircle
######Callbacks
def shutdownHook():
    Local.goal_reached = 1
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = 0 #make slower at last point
    Local.cmd_vel_publisher.publish(twist)
def robotPosCallback(robot):
    quat = [robot.pose.pose.orientation.x,robot.pose.pose.orientation.y,
    robot.pose.pose.orientation.z,robot.pose.pose.orientation.w]

    Local.robot_pos = np.array(
        [robot.pose.pose.position.y/ Local.costmap_resolution, 
        robot.pose.pose.position.x/Local.costmap_resolution,
        tf.transformations.euler_from_quaternion(quat)[2]])
    #if Local.debug:
        #rospy.loginfo(f"New pos = {Local.new_pos}")

def pathCallback(path):################Доделать
    Local.targets.clear()
    Local.new_targets.clear()
    Local.current_target = 0 
    #rospy.loginfo_once(f"Got path, poses = {path.poses}")
    for pose in path.poses:
        quat = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        target = np.array([pose.pose.position.y,pose.pose.position.x,tf.transformations.euler_from_quaternion(quat)[2]])
        Local.new_targets.append(target)
    Local.parseTargets()
    Local.goal_reached = 0
    rospy.loginfo(f'Goal Set!')
def costmapCallback(costmap):
    Local.costmap_resolution = costmap.info.resolution
    Local.costmap_height = costmap.info.height
    Local.costmap_width = costmap.info.width
    rospy.loginfo_once(f"Got new map, height = {Local.costmap_height}, width= {Local.costmap_width}")
    Local.costmap= np.reshape(costmap.data,(Local.costmap_height, Local.costmap_width))
def costmapUpdateCallback(update):
    origin_x = update.x
    origin_y = update.y
    for y in range (update.height):
        for x in range (update.width):
            Local.costmap[origin_y + y][origin_x + x] = update.data[x+y]
######/Callbacks
#Field :   204x304 cm
class Local():
    
    roslib.load_manifest('ebobot')
    #Params
    #Features
    rotate_at_end = rospy.get_param('local_planer/rotate_at_end', 1)
    get_lowest_cost = rospy.get_param('local_planer/get_lowest_cost', 0)
    delta_thetas_enable =  rospy.get_param('local_planer/delta_thetas_enable', 0)
    cost_coeff_enable = rospy.get_param('local_planer/cost_coeff_enable', 0)
    path_coeff_enable = rospy.get_param('local_planer/path_coeff_enable', 1)
    debug = rospy.get_param('local_planer/debug', 1)
    #/Features
    #Turn params
    turn_threshhold = rospy.get_param('local_planer/turn_threshhold', 0.2)
    cells_per_radian = rospy.get_param('local_planer/cells_per_radian', 5)
    turn_coeff = rospy.get_param('local_planer/cells_per_radian', 0.5)
    #
   
    
    static_coeff = rospy.get_param('local_planer/static_coeff', 0.2)
    min_path_coeff = rospy.get_param('local_planer/min_path_coeff', 0.1)
    path_speed_coeff = rospy.get_param('local_planer/path_speed_coeff', 1)
    cost_threshhold = rospy.get_param('local_planer/cost_threshhold', 10000) #100 are walls, then there is inflation
    num_of_steps_between_clss = rospy.get_param('local_planer/num_of_steps_between_clss', 4)
    update_rate = rospy.get_param('local_planer/update_rate', 20) # in Hz
    cost_speed_coeff = rospy.get_param('local_planer/cost_speed_coeff', 0.0002)
    threshhold = rospy.get_param('local_planer/threshhold', 2) #in cells
    
    
    
    
    num_of_circles = rospy.get_param('local_planer/num_of_circles', 2)
    circles_dist = rospy.get_param('local_planer/circles_dist', 1) #in cells
    circles_step_radians_resolution = rospy.get_param('local_planer/circles_step_radians_resolution', 6) #number of points on each circle
    #### Params for footprint cost calc
    #calculate_base_cost = rospy.get_param('local_planer/calculate_base_cost', 1)
    base_footprint_radius = rospy.get_param('local_planer/base_footprint_radius', 0.20) #optional
    safe_footprint_radius = rospy.get_param('local_planer/safe_footprint_radius', 0.30)
    footprint_calc_step_radians_resolution = rospy.get_param('local_planer/footprint_calc_step_radians_resolution', int(safe_footprint_radius*50*6)) #number of points on circle to check cost
    #### /Params for footprint cost calc
    #/Params

    #Topics
    #rviz_point_topic = rospy.get_param('local_planer/rviz_topic', 'local_points')
    path_subscribe_topic =  rospy.get_param('local_planer/path_subscribe_topic', '/global_path')
    costmap_topic = rospy.get_param('local_planer/costmap_topic', '/costmap')
    costmap_update_topic = rospy.get_param('local_planer/costmap_update_topic', '/costmap_updates')
    robot_pos_topic = rospy.get_param('local_planer/robot_pos_topic', '/odom')
    cmd_vel_topic = rospy.get_param('local_planer/cmd_vel_topic', '/cmd_vel')

    ######
    #point_publisher = rospy.Publisher(rviz_point_topic, Marker, queue_size = 10)
    rviz_broadcaster = tf.TransformBroadcaster()
    costmap_update_subscriber = rospy.Subscriber(costmap_update_topic, OccupancyGridUpdate, costmapUpdateCallback)
    costmap_subscriber = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback)
    path_subscriber = rospy.Subscriber(path_subscribe_topic, Path, pathCallback)
    cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 25)
    robot_pos_subscriber = rospy.Subscriber(robot_pos_topic, Odometry, robotPosCallback)
    #/Topics

    #cls values
    actual_target = []
    max_dist = 0
    skipped = 0
    goal_reached = 1
    new_targets = []
    costmap_resolution = 0.02
    new_pos = np.array([0,0,0])
    robot_pos = np.array([0,0,0])
    default_costmap_list = [[0]*101 for _ in range(151)]
    costmap = np.array(default_costmap_list)
    costmap_width = 151
    costmap_height = 101
    cost_coords_list = dCoordsOnCircle(safe_footprint_radius//costmap_resolution,footprint_calc_step_radians_resolution)
    targets = []
    current_target = 0
    cost_check_poses = []
    for n in range(1,num_of_circles+1):
        for x,y in dCoordsOnCircle(n*circles_dist,n*circles_step_radians_resolution):
            cost_check_poses.append((x,y))
    #/cls values
    ##############################deltaCoordsPrecalc
    @staticmethod
    def recalcCostCoordsFromRadius(radius):
        Local.cost_coords_list = [(x,y) for x,y in dCoordsOnCircle(radius/Local.costmap_resolution,Local.footprint_calc_step_radians_resolution)]
    #############################/deltaCoordsPrecalc

    @classmethod
    def parseTargets(cls):
        new_parsed_targets = []
        if cls.debug:
            rospy.loginfo(f"Parsing targets...")
        current_theta = cls.robot_pos[2]
        final_target = cls.new_targets.pop()
        if cls.delta_thetas_enable:
            delta_theta = (final_target[2] - current_theta) / (len(cls.targets) + 1)
        elif cls.rotate_at_end:
            delta_theta = current_theta
        # else:
        #     delta_theta = final_target[2]
        for num,target in enumerate(cls.new_targets):
            new_parsed_targets.append(np.append(target[:2],delta_theta * num))
        new_parsed_targets.append(final_target)
        cls.targets = new_parsed_targets
        cls.max_dist = np.linalg.norm(cls.targets[-1] - cls.targets[0])
        if cls.debug:
            rospy.loginfo(f"Parsed targets = {cls.targets}")
    @classmethod
    def getCost(cls,pose):
        curr_y,curr_x = pose[0], pose[1]
        cost = 0
        #rospy.loginfo_once(f"Cost coords list{Local.cost_coords_list}")
        for y,x in cls.cost_coords_list:
            new_x, new_y = curr_x+x, curr_y+y
            if cls.debug:
                rospy.loginfo_once(f"getting cost from x = {new_x}, y = {new_y}, w-h = {cls.costmap_width,cls.costmap_height}")
            if 0<= new_x < Local.costmap_width and 0<= new_y < Local.costmap_height:
                cost += Local.costmap[int(new_y)][int(new_x)]
            else:
                cost += 100
        return cost
    
    @classmethod
    def cmdVel(cls,target,speed_coeff):       #speed ranges from 0 to 1
        if speed_coeff > 1:
            speed_coeff = 1
        twist = Twist()
        #move =  target/np.linalg.norm(target)*speed_coeff#make param
        
        twist.linear.x = target[1] #forward (x)
        twist.linear.y = target[0] #left (y)
        twist.angular.z = target[2] #counterclockwise
        Local.cmd_vel_publisher.publish(twist)
    ###############################
    @classmethod
    def remapToLocal(cls,vect):
        #norm = np.linalg.norm(vect[:2])
        #curr_y,curr_x = cls.robot_pos[0],cls.robot_pos[1]
        curr_ang = cls.robot_pos[2]
        new_vect = turnVect((vect[0],vect[1]),curr_ang)
        dist = np.linalg.norm(new_vect[:2])  
        if cls.rotate_at_end:
            if cls.current_target == len(cls.targets)-1:
                diff =  (vect[2]-curr_ang)
                turn = diff/abs(diff)*  cls.turn_coeff
            turn = 0
        else:
            turn = (vect[2]-curr_ang)/dist * cls.cells_per_radian *  cls.turn_coeff
        result = np.array([new_vect[0] /dist,  new_vect[1]/dist,   turn])
        #if cls.debug:
            #rospy.loginfo(f"Remapping... {vect=},{result=}")
            #pass
        return (result[0],result[1],result[2])

    #########################
    @classmethod
    def getPathSpdCoeff(cls):    
        final_coeff = cls.max_dist - np.linalg.norm(cls.targets[-1] - cls.robot_pos)/cls.max_dist * cls.path_speed_coeff
        if final_coeff < cls.min_path_coeff:
            final_coeff = cls.min_path_coeff
        elif final_coeff > 1:
            final_coeff = 1
        #rospy.loginfo_once(f"Fetched speed coeff from dist to goal = {final_coeff}")
        return final_coeff


    @classmethod
    def fetchPoint(cls,current_pos):     #dist to target should be checked in updateDist()
        
        #current = cls.robot_pos 
        if cls.skipped + cls.current_target >= len(cls.targets):
            rospy.loginfo(f"Goal failed! Sending Stop!")
            cls.goal_reached = 1
            cls.skipped = 0
            shutdownHook()
            return current_pos
        target = cls.targets[cls.current_target+cls.skipped]
        if cls.debug:
            rospy.loginfo(f"Fetching point with curr = {current_pos}, target = {target}")
        point = target
        min_cost = cls.getCost(target)
        point_cost = min_cost
        if cls.get_lowest_cost:
            for y,x in cls.cost_check_poses:
                pose = np.array([target[0] + y, target[1] + x, target[2]])
                curr_cost = cls.getCost(pose)
                if curr_cost < min_cost:
                    point = pose
                    point_cost = curr_cost 
                    min_cost = curr_cost
            if cls.debug:
                rospy.loginfo(f"Best subpoint = {point}({point_cost})")
            if point_cost > cls.cost_threshhold: 
                if cls.debug:
                    rospy.loginfo(f"Point failed cost check({point_cost})! Recursing...")
                cls.skipped += 1
                #cls.current_target += 1
                #cls.fetchPoint(current, target)e
                return cls.fetchPoint()
        else:
            if cls.debug:
                rospy.loginfo(f"Fetching target {point =}\ncurr = {current_pos}")
            cls.skipped = 0 
            cls.current_target += 1
            return point 
       
    ####################################################################       
    # @classmethod
    # def updatePos(cls):
    #     Local.robot_pos = Local.new_pos
    @classmethod
    def checkPos(cls):
        return np.linalg.norm(cls.robot_pos[:2] - cls.actual_target[:2]) > cls.threshhold
    @classmethod
    def checkTurn(cls):
        return (cls.robot_pos[2] - cls.actual_target[2]) > cls.turn_threshhold
    @classmethod
    def updateTarget(cls):
        #Local.updatePos()
        #current_pos = cls.robot_pos
        if Local.debug:
            rospy.loginfo(f"robot pos {cls.robot_pos}")
        if cls.debug:
            rospy.loginfo(f'Updating target {cls.current_target},\n current = {cls.robot_pos} (max targs = {len(cls.targets)})')
        if cls.current_target < len(cls.targets)-1:
            cls.actual_target =  cls.fetchPoint(cls.robot_pos)
        elif cls.current_target == len(cls.targets)-1:
            cls.actual_target = cls.targets[cls.current_target]
        else:
            Local.goal_reached = 1
            shutdownHook()
        if cls.debug:
            rospy.loginfo(f'Riding to {cls.actual_target}')
        
        while cls.checkPos() and not rospy.is_shutdown():
            #Local.updatePos()
            speed_coeff = 1
            if cls.cost_coeff_enable:
                speed_coeff = speed_coeff * cls.getCost(cls.actual_target)
            if cls.path_coeff_enable:
                speed_coeff = speed_coeff * cls.getPathSpdCoeff()
            cmd_target = cls.remapToLocal(cls.actual_target-cls.robot_pos) ###ADJUSTS GLOBAL COMAND TO LOCAL
            cls.cmdVel(cmd_target, speed_coeff*cls.static_coeff)#make slower at last point
            rospy.sleep(1/cls.update_rate)

        return
    ####################################################
   
    

# import os
# import cv2
def main():
    rospy.init_node('local_planer')
    rospy.on_shutdown(shutdownHook)
    rate = rospy.Rate(Local.update_rate)
    while not rospy.is_shutdown():
        if not Local.goal_reached:
            if np.linalg.norm(Local.robot_pos - Local.targets[-1]) < Local.threshhold:
                rospy.loginfo(f'Goal reached!')
                Local.goal_reached = 1
                shutdownHook()
                #Local.current_target = len(Local.targets)-1
            else:
                #Local.updatePos()
                Local.updateTarget()
        rate.sleep()

    # os.chdir("/home/alexej/catkin_ws/src/ebobot/nodes/costmap")
    # cv2.imwrite("recieved_map.png", Local.costmap)
    # rospy.sleep(5)

if __name__=="__main__":
    main()
