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
from dorlib import dCoordsInRad, dCoordsOnCircle
######Callbacks
def shutdownHook():
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = 0 #make slower at last point
    Local.cmd_vel_publisher.publish(twist)
def robotPosCallback(pose):
    quat = [pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w]
    Local.robot_pos = np.array([pose.pose.pose.position.y/ Local.costmap_resolution, pose.pose.pose.position.x/Local.costmap_resolution,tf.transformations.euler_from_quaternion(quat)[2]])
    #rospy.loginfo(f"robot pos {Local.robot_pos}")

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
    delta_thetas_enable =  rospy.get_param('local_planer/delta_thetas_enable', 0)
    cost_coeff_enable = rospy.get_param('local_planer/cost_coeff_enable', 0)
    path_coeff_enable = rospy.get_param('local_planer/path_coeff_enable', 0)
    debug = rospy.get_param('local_planer/debug', 1)
    #/Features
    


    rviz_point_topic = rospy.get_param('local_planer/rviz_topic', 'local_points')
    rviz_topic = rospy.get_param('local_planer/rviz_topic', 'rviz_local_path')
    
    static_coeff = rospy.get_param('local_planer/static_coeff', 0.6)
    min_path_coeff = rospy.get_param('local_planer/min_path_coeff', 0.3)
    path_speed_coeff = rospy.get_param('local_planer/path_speed_coeff', 1)
    cost_threshhold = rospy.get_param('local_planer/cost_threshhold', 10000) #100 are walls, then there is inflation
    num_of_steps_between_clss = rospy.get_param('local_planer/num_of_steps_between_clss', 4)
    update_rate = rospy.get_param('local_planer/update_rate', 10) # in Hz
    cost_speed_coeff = rospy.get_param('local_planer/cost_speed_coeff', 0.0002)
    threshhold = rospy.get_param('local_planer/threshhold', 3) #in cells
    
    
    
    
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
    path_subscribe_topic =  rospy.get_param('local_planer/path_subscribe_topic', '/global_path')
    costmap_topic = rospy.get_param('local_planer/costmap_topic', '/costmap')
    costmap_update_topic = rospy.get_param('local_planer/costmap_update_topic', '/costmap_updates')
    robot_pos_topic = rospy.get_param('local_planer/robot_pos_topic', '/odom')
    cmd_vel_topic = rospy.get_param('local_planer/cmd_vel_topic', '/cmd_vel')

    ######
    point_publisher = rospy.Publisher(rviz_point_topic, Marker, queue_size = 10)
    rviz_broadcaster = tf.TransformBroadcaster()
    rviz_publisher = rospy.Publisher(rviz_topic, Path, queue_size = 10)
    costmap_update_subscriber = rospy.Subscriber(costmap_update_topic, OccupancyGridUpdate, costmapUpdateCallback)
    costmap_subscriber = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback)
    path_subscriber = rospy.Subscriber(path_subscribe_topic, Path, pathCallback)
    cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 25)
    robot_pos_subscriber = rospy.Subscriber(robot_pos_topic, Odometry, robotPosCallback)
    #/Topics

    #cls values
    actual_target = []
    
    skipped = 0
    goal_reached = 1
    new_targets = []
    costmap_resolution = 0.02
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
        else:
            delta_theta = current_theta
        
        for num,target in enumerate(cls.new_targets):
            new_parsed_targets.append(np.append(target[:2],delta_theta * num))
        new_parsed_targets.append(final_target)
        cls.targets = new_parsed_targets
        if cls.debug:
            rospy.loginfo(f"Parsed targets = {cls.targets}")
    @staticmethod
    def getCost(pose):
        curr_y,curr_x = pose[0], pose[1]
        cost = 0
        #rospy.loginfo_once(f"Cost coords list{Local.cost_coords_list}")
        for x,y in Local.cost_coords_list:
            new_x, new_y = curr_x+x, curr_y+y
            rospy.loginfo_once(f"getting cost from x = {new_x}, y = {new_y}")
            if new_x < Local.costmap_width and new_x>=0 and  new_y < Local.costmap_height and new_y>=0:
                cost += Local.costmap[int(round(new_x))][int(round(new_y))]
            else:
                cost += 100
        return cost
    
    @classmethod
    def cmdVel(cls,target,speed_coeff):       #speed ranges from 0 to 1
        if speed_coeff > 1:
            speed_coeff = 1
        twist = Twist()
        move =  target/np.linalg.norm(target)*speed_coeff#make param
        if cls.debug:
            rospy.loginfo(f"Updating /cmd_vel to {move}, speed_coeff = {speed_coeff}")
        twist.linear.y = move[0]
        twist.linear.x = move[1]
        twist.angular.z = move[2] 
        Local.cmd_vel_publisher.publish(twist)
    ###############################
    @classmethod
    def remapToLocal(cls,vect):
        norm = np.linalg.norm(vect[:2])
        curr_ang = cls.robot_pos[2]
        result = (vect[0] +norm * cos(curr_ang),vect[1] + norm * sin(curr_ang),vect[2]-curr_ang)
        return result

    #########################
    @staticmethod
    def getPathSpdCoeff():
        max_targets = len(Local.targets)
        coeff = abs(Local.current_target - max_targets/2) / (max_targets/2)
        final_coeff = coeff * Local.path_speed_coeff
        if final_coeff < Local.min_path_coeff:
            final_coeff = Local.min_path_coeff
        #rospy.loginfo_once(f"Fetched speed coeff from dist to goal = {final_coeff}")
        return final_coeff


    @classmethod
    def fetchPoint(cls):     #dist to target should be checked in updateDist()
        
        current = cls.robot_pos 
        if cls.skipped + cls.current_target >= len(cls.targets):
            rospy.loginfo(f"Goal failed! Sending Stop!")
            cls.goal_reached = 1
            cls.skipped = 0
            cls.current = len(cls.targets)
            return current 
        target = cls.targets[cls.current_target+cls.skipped]
        if cls.debug:
            rospy.loginfo(f"Fetching point with curr = {current}, target = {target}")
            #rospy.loginfo(f"Skipped = {cls.skipped}, subtarget = {cls.subtarget}, target = {cls.current_target}") 
        point = target
        min_cost = cls.getCost(target)
        point_cost = min_cost
        rospy.loginfo_once(f"Cost list = {cls.cost_check_poses}")
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
                rospy.loginfo(f"Fetching target")
            cls.skipped = 0 
            cls.current_target += 1
            return point 
       
    ####################################################################       
    @classmethod
    def updateTarget(cls):
        current_pos = cls.robot_pos
        rospy.loginfo(f'Updating target {cls.current_target}, current = {current_pos} (max targs = {len(cls.targets)})')
        #target = cls.targets[cls.current_target]
        if cls.current_target < len(cls.targets)-1:
            actual_target =  cls.fetchPoint()
        elif cls.current_target == len(cls.targets)-1:
            actual_target = cls.targets[cls.current_target]
        else:
            Local.goal_reached = 1
            shutdownHook()
        #cls.pubPath()
        rospy.loginfo(f'Riding to {actual_target}')
        while np.linalg.norm(cls.robot_pos - actual_target) > cls.threshhold and not rospy.is_shutdown() and not cls.goal_reached:
            speed_coeff = 1
            if cls.cost_coeff_enable:
                speed_coeff = speed_coeff * cls.getCost(cls.actual_target)
            if cls.path_coeff_enable:
                speed_coeff = speed_coeff * cls.getPathSpdCoeff()
            #cmd_target = cls.actual_target - cls.robot_pos
            cmd_target = cls.remapToLocal(actual_target-current_pos) ###ADJUSTS GLOBAL COMAND TO LOCAL
            cls.cmdVel(cmd_target, speed_coeff*cls.static_coeff)#make slower at last point
            rospy.sleep(1/cls.update_rate)
        cls.current_target +=1
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
                #Local.current_target = len(Local.targets)-1
            else: 
                Local.updateTarget()
        rate.sleep()

    # os.chdir("/home/alexej/catkin_ws/src/ebobot/nodes/costmap")
    # cv2.imwrite("recieved_map.png", Local.costmap)
    # rospy.sleep(5)

if __name__=="__main__":
    main()
