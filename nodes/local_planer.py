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
    Local.robot_pos = np.array([pose.pose.pose.position.x/ Local.costmap_resolution, pose.pose.pose.position.y/Local.costmap_resolution,tf.transformations.euler_from_quaternion(quat)[2]])
    rospy.loginfo(f"robot pos {Local.robot_pos}")

def pathCallback(path):################Доделать
    Local.targets.clear()
    Local.new_targets.clear()
    Local.current_target = 0 
    #rospy.loginfo_once(f"Got path, poses = {path.poses}")
    for pose in path.poses:
        quat = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        target = np.array([pose.pose.position.x,pose.pose.position.y,tf.transformations.euler_from_quaternion(quat)[2]])
        Local.new_targets.append(target)
    Local.parseTargets()
    Local.goal_reached = 0
    rospy.loginfo(f'Goal Set!')
def costmapCallback(costmap):
    Local.costmap_resolution = costmap.info.resolution
    Local.costmap_height = costmap.info.height
    Local.costmap_width = costmap.info.width
    rospy.loginfo_once(f"Got new map, height = {Local.costmap_height}, width= {Local.costmap_width}")
    Local.costmap= np.rot90(np.reshape(costmap.data,(Local.costmap_height, Local.costmap_width))   ,3)
def costmapUpdateCallback(update):
    origin_x = update.x
    origin_y = update.y
    for x in range (update.width):
        for y in range (update.height):
            Local.costmap[origin_x + x][origin_y + y] = update.data[x+y]
######/Callbacks
#Field :   204x304 cm
class Local():
    
    roslib.load_manifest('ebobot')
    #Params
    delta_thetas_enable =  rospy.get_param('local_planer/min_speed_coeff', 0)
    min_speed_coeff = rospy.get_param('local_planer/min_speed_coeff', 0.3)
    path_speed_coeff = rospy.get_param('local_planer/path_speed_coeff', 1)
    cost_threshhold = rospy.get_param('local_planer/cost_threshhold', 1000) #100 are walls, then there is inflation
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
    num_of_circles = rospy.get_param('local_planer/num_of_circles', 2)
    circles_dist = rospy.get_param('local_planer/circles_dist', 1) #in cells
    circles_step_radians_resolution = rospy.get_param('local_planer/circles_step_radians_resolution', 6) #number of points on each circle
    #### Params for footprint cost calc
    #calculate_base_cost = rospy.get_param('local_planer/calculate_base_cost', 1)
    base_footprint_radius = rospy.get_param('local_planer/base_footprint_radius', 0.20) #optional
    safe_footprint_radius = rospy.get_param('local_planer/safe_footprint_radius', 0.30)
    footprint_calc_step_radians_resolution = rospy.get_param('local_planer/footprint_calc_step_radians', int(safe_footprint_radius*50*6)) #number of points on circle to check cost
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
    actual_target = []
    current_max_subtargets = 0
    subtarget = 0
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
    #/global values
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
            delta_theta = 0
        
        for num,target in enumerate(cls.new_targets):
            new_parsed_targets.append(np.append(target[:2],delta_theta * num))
        new_parsed_targets.append(final_target)
        cls.targets = new_parsed_targets
        if cls.debug:
            rospy.loginfo(f"Parsed targets = {cls.targets}")
    @staticmethod
    def getCost(pose):
        curr_x,curr_y = pose[0], pose[1]
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
    @classmethod
    def fetchPoint(cls):     #dist to target should be checked in updateDist()
        current = cls.robot_pos 
        target = cls.targets[cls.current_target]
        if cls.debug:
            rospy.loginfo(f"Fetching point with curr = {current}, target = {target}")
            rospy.loginfo(f"Skipped = {cls.skipped}, subtarget = {cls.subtarget}, target = {cls.current_target}")
        cls.current_max_subtargets = cls.num_of_steps_between_globals*(cls.skipped+1)
        cls.subtarget += 1
        if cls.subtarget:
            curr_targ = (target - current) / (cls.current_max_subtargets * cls.subtarget)
        
        min_cost = 0
        if cls.current_target == len(cls.targets):
            point = curr_targ
            point_cost = 0
            rospy.loginfo("Last point!")
        else:
            point = curr_targ
            point_cost = cls.getCost(curr_targ)
            rospy.loginfo_once(f"Cost list = {cls.cost_check_poses}")
            for x,y in cls.cost_check_poses:
                pose = np.array([curr_targ[0] + x, curr_targ[1] + y, curr_targ[2]])
                curr_cost = cls.getCost(pose)
                if curr_cost < min_cost:
                    point = pose
                    point_cost = curr_cost
                    min_cost = curr_cost
            if cls.debug:
                rospy.loginfo(f"Best subpoint = {point}")
        if point_cost > cls.cost_threshhold: 
            if cls.debug:
                rospy.loginfo(f"Point failed cost check! Recursing...")
            if cls.current_target== len(cls.targets):
                rospy.loginfo(f"Goal failed! Sending Stop!")
                cls.actual_target =  current
                return -1
            cls.subtarget = 0
            cls.skipped += 1
            #cls.current_target += 1
            #cls.fetchPoint(current, target)
            return 0
        elif cls.subtarget == cls.current_max_subtargets:
            if cls.debug:
                rospy.loginfo(f"Fetching full target")
            cls.skipped = 0
            cls.subtarget = 0
            cls.current_target += 1
            cls.actual_target = point 
            return 1
        else:
            if cls.debug:
                rospy.loginfo(f"Fetching subtarget")
            cls.subtarget += 1
            cls.actual_target = point
            return 1
           
    @classmethod
    def updateTarget(cls):
        cls.current_target +=1
        current_pos = cls.robot_pos
        rospy.loginfo(f'Updating target {cls.current_target}, current = {current_pos} (max targs = {len(cls.targets)})')
        target = cls.targets[cls.current_target]
        if cls.fetchPoint():
            rospy.loginfo(f'Riding to {cls.actual_target}')
        else:
            cls.updateTarget()
        while np.linalg.norm(cls.robot_pos - cls.actual_target) > cls.threshhold:
            cost_speed_coeff = cls.cost_speed_coeff*cls.getCost(target[0],target[1])
            cls.cmdVel(target, cost_speed_coeff * cls.getPathSpdCoeff())
            #rospy.sleep(1/cls.update_rate)
        
 


def main():
    rospy.init_node('local_planer')
    rate = rospy.Rate(Local.update_rate)
    #Local.precalcCostCoordsFromRadius()
    #if Local.debug:
        #rospy.loginfo(f"Cost coords = {Local.cost_coords_list}")
    while not rospy.is_shutdown():
      
        if not Local.goal_reached:
            #rospy.loginfo(f'Target = {Local.targets[-1]}')
            #current_target = Local.targets[Local.current_target]
                #if current_target == len(Local.targets):
            if np.linalg.norm(Local.robot_pos - Local.targets[-1]) < Local.threshhold:
                rospy.loginfo(f'Goal reached!')
                Local.goal_reached = 1
            else: #np.linalg.norm(Local.robot_pos-current_target) > Local.threshhold: #make param
                Local.updateTarget()
    rate.sleep()

if __name__=="__main__":
    main()
