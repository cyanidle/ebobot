#!/usr/bin/env python3
from cmath import acos
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
from std_msgs.msg import String, Int8, Float32
from std_srvs.srv import SetBool, SetBoolRequest
######
from dorlib import turnVect, dCoordsOnCircle
import markers
####
roslib.load_manifest('ebobot')
rospy.init_node('local_planer')
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
    Local.robot_twist = (Local.twist_amplify_coeff * 
    np.array([robot.twist.twist.linear.y,
    robot.twist.twist.linear.x,
    robot.twist.twist.angular.z]) /
    Local.costmap_resolution
    )
    #rospy.logwarn(f"{robot.twist.twist.linear.x = }")
    Local.robot_pos = np.array(
        [robot.pose.pose.position.y/ Local.costmap_resolution, 
        robot.pose.pose.position.x/Local.costmap_resolution,
        tf.transformations.euler_from_quaternion(quat)[2]])
    #if Local.debug:
        #rospy.loginfo(f"New pos = {Local.new_pos}")
def changeCostCallback(fl):
    cost = fl.data
    if not Local.global_cost_relation:
        Local.global_cost_relation = Local.cost_threshhold/cost
    else:
        Local.cost_threshhold = cost * Local.global_cost_relation
        rospy.logwarn(f"LOCAL: New global cost {cost}, threshhold {Local.cost_threshhold}")
def pathCallback(path):################Доделать
    #Local.targets.clear()
    Local.new_targets.clear()
    Local.goal_reached = 1
    #Local.current_target = 0 
    #rospy.loginfo_once(f"Got path, poses = {path.poses}")
    for pose in path.poses:
        quat = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        target = np.array([pose.pose.position.y,pose.pose.position.x,tf.transformations.euler_from_quaternion(quat)[2]])
        Local.new_targets.append(target)
    Local.last_target = target
    Local.parseTargets()
    if not len(Local.new_targets): # == 1 and np.linalg.norm(Local.targets[-1][:2] - Local.robot_pos[:2]) < Local.threshhold):
        rospy.logerr("LOCAL SHUTDOWN HOOK ACTIVATED, GOAL UNREACHABLE!")
        shutdownHook()
    
def costmapCallback(costmap):
    Local.costmap_resolution = costmap.info.resolution
    Local.costmap_height = costmap.info.height
    Local.costmap_width = costmap.info.width
    rospy.loginfo_once(f"Got new map, height = {Local.costmap_height}, width= {Local.costmap_width}")
    Local.costmap= np.reshape(costmap.data,(Local.costmap_height, Local.costmap_width))
def costmapUpdateCallback(update): #not used currently
    origin_x = update.x
    origin_y = update.y
    for y in range (update.height):
        for x in range (update.width):
            Local.costmap[origin_y + y][origin_x + x] = update.data[x+y]
######/Callbacks
#Field :   204x304 cm
class Local():
    
    
    #Params
    #Features
    rotate_at_end = rospy.get_param('~rotate_at_end', 1)
    
    get_lowest_cost = rospy.get_param('~get_lowest_cost', 0)
    cost_coeff_enable = rospy.get_param('~cost_coeff_enable', 0)
    path_coeff_enable = rospy.get_param('~path_coeff_enable', 0)
    debug = rospy.get_param('~debug', 1)
    #/Features
    #Turn params
    pause_before_turn = rospy.get_param('~pause_before_turn', 0.2) #seconds
    turn_threshhold = rospy.get_param('~turn_threshhold', 0.05) 
    cells_per_radian = rospy.get_param('~cells_per_radian', 5)
    turn_coeff = rospy.get_param('~turn_coeff', 0.25)
    min_turn_coeff = rospy.get_param('~min_turn_coeff', 0.28) #final!
    #
   
    #speed coeffs
    full_path_coeff_dist = rospy.get_param('~full_path_coeff_dist', 60) #dist from target in cells at which robot goes full spd
    static_coeff = rospy.get_param('~static_coeff', 0.4)
    min_coeff = rospy.get_param('~min_coeff', 0.2) #Final!!
    path_speed_coeff = rospy.get_param('~path_speed_coeff', 0.3) #В тугриках
    #planer
    cost_threshhold = rospy.get_param('~cost_threshhold', 10000) #100 are walls, then there is inflation
    
    update_rate = rospy.get_param('~update_rate', 20) # in Hz
    cost_speed_coeff = rospy.get_param('~cost_speed_coeff', 2)
    max_cost_speed_coeff = rospy.get_param('~max_cost_speed_coeff', 2)
    threshhold = rospy.get_param('~threshhold', 2) #in cells
    
    skip_thresh=rospy.get_param('~skip_thresh', 4) #max skipped targets (from cost) before fail
    
    
    num_of_circles = rospy.get_param('~num_of_circles', 2)
    circles_dist = rospy.get_param('~circles_dist', 1) #in cells
    circles_step_radians_resolution = rospy.get_param('~circles_step_radians_resolution', 6) #number of points on each circle
    
    #### Params for footprint cost calc
    base_footprint_radius = rospy.get_param('~base_footprint_radius', 0.20) #optional
    safe_footprint_radius = rospy.get_param('~safe_footprint_radius', 0.30)
    footprint_calc_step_radians_resolution = rospy.get_param('~footprint_calc_step_radians_resolution', int(safe_footprint_radius*50*6)) #number of points on circle to check cost
    #### /Params for footprint cost calc
    cost_threshhold *= footprint_calc_step_radians_resolution #!!!!!!!!!!!!!!!!!!!!!!!
    twist_amplify_coeff = rospy.get_param('~twist_amplify_coeff', 1)
    inertia_compensation_coeff = rospy.get_param('~inertia_compensation_coeff', 0.8)
    #/Params

    #Topics
    #rviz_point_topic = rospy.get_param('~rviz_topic', 'local_points')
    change_cost_service_echo_name = f"{rospy.get_param('/global/change_cost_service_name', 'change_cost_service')}_echo"
    status_publish_topic = rospy.get_param('~status_publish_topic', 'planers/local/status')
    path_subscribe_topic =  rospy.get_param('~path_subscribe_topic', 'planers/global/path')
    costmap_topic = rospy.get_param('~costmap_topic', 'costmap_server/costmap')
    costmap_update_topic = rospy.get_param('~costmap_update_topic', 'costmap_server/updates')
    robot_pos_topic = rospy.get_param('~robot_pos_topic', 'odom')
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
    disable_adjust_sec_topic = rospy.get_param('~disable_adjust_sec_topic', 'disable_adjust_sec')
    disable_adjust_sec_time = rospy.get_param('~disable_adjust_sec_time', 4)
    adjust_toggle_name  = rospy.get_param('~adjust_toggle_name', "adjust_toggle_service")
     ###
    use_timed_adj_disable = rospy.get_param('~use_timed_adj_disable', 0)
    _toggle_proxy = rospy.ServiceProxy(adjust_toggle_name, SetBool)
    if rotate_at_end:
        disable_adjust_publisher = rospy.Publisher(disable_adjust_sec_topic, Int8, queue_size = 4)
    ######
    #point_publisher = rospy.Publisher(rviz_point_topic, Marker, queue_size = 10)
    cost_sub = rospy.Subscriber(change_cost_service_echo_name, Float32, changeCostCallback)
    rviz_broadcaster = tf.TransformBroadcaster()
    status_publisher = rospy.Publisher(status_publish_topic, String, queue_size = 5)
    costmap_update_subscriber = rospy.Subscriber(costmap_update_topic, OccupancyGridUpdate, costmapUpdateCallback)
    costmap_subscriber = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback)
    path_subscriber = rospy.Subscriber(path_subscribe_topic, Path, pathCallback)
    cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 25)
    robot_pos_subscriber = rospy.Subscriber(robot_pos_topic, Odometry, robotPosCallback)
    #/Topics

    #cls values
    global_cost_relation = 0
    robot_twist = np.array([0,0,0])
    last_target = []
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
        if not cls.rotate_at_end:
            delta_theta = (final_target[2] - current_theta) / (len(cls.targets) + 1)
        else:
            delta_theta = 0
        # else:
        #     delta_theta = final_target[2]
        min_dist = 100
        _current_target = 0
        for num,target in enumerate(cls.new_targets):
            new_parsed_targets.append(np.append(target[:2],delta_theta * num))
            dist = np.linalg.norm(target[:2] - (cls.robot_pos[:2]+cls.robot_twist[:2]))
            if dist < min_dist:
                min_dist = dist
                _current_target = int(num)
        cls.targets = new_parsed_targets #IMPORTANT
        cls.current_target = _current_target
        #cls.actual_target = cls.robot_pos + cls.robot_twist
        new_parsed_targets.append(final_target)  
        if cls.debug:
            rospy.loginfo(f"Parsed targets = {cls.targets}")
        cls.goal_reached = 0
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
    def cmdVel(cls,target,speed_coeff = 1):       #speed ranges from 0 to 1
        if speed_coeff > 1:
            speed_coeff = 1
        twist = Twist()
        #move =  target/np.linalg.norm(target)*speed_coeff#make param
        twist.linear.x = target[1]*speed_coeff #forward (x)
        twist.linear.y = target[0]*speed_coeff #left (y)
        twist.angular.z = target[2]*speed_coeff #counterclockwise
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
        if len(cls.targets):
            curr_dist = np.linalg.norm(cls.targets[-1][:2] - cls.robot_pos[:2])
            final_coeff = curr_dist/cls.full_path_coeff_dist * cls.path_speed_coeff
            #rospy.loginfo(f"{final_coeff = }|{curr_dist = }|{cls.full_path_coeff_dist = }")
        else:
            final_coeff = 0
        #rospy.loginfo(f"{final_coeff = }, {cls.full_path_coeff_dist = },{np.linalg.norm(cls.targets[-1][:2] - cls.robot_pos[:2]) = }")
        if final_coeff < cls.min_coeff:
            final_coeff = cls.min_coeff
        if final_coeff > 1:
            final_coeff = 1
        #rospy.loginfo_once(f"Fetched speed coeff from dist to goal = {final_coeff}")
        return final_coeff
    @classmethod
    def checkTurn(cls): 
        #rospy.loginfo(f"Checking turn {cls.getRadNorm(cls.robot_pos[2])=}{cls.getRadNorm(cls.last_target[2])=}...")
        return abs(cls.getRadNorm(cls.last_target[2]) - cls.getRadNorm(cls.robot_pos[2])) > cls.turn_threshhold
    @staticmethod
    def getRadNorm(rad):
        if rad >= 0:
            return rad
        else:
            return 6.283 + rad
    @classmethod
    def rotateAtEnd(cls):
        rospy.logwarn(f"Rotating...")
        #shutdownHook()
        cls.cmdVel((0,0,0),1)
        rospy.sleep(cls.pause_before_turn)
        _toggle_resp_f = None
        if cls.use_timed_adj_disable:
            cls.disable_adjust_publisher.publish(Int8(cls.disable_adjust_sec_time))
        else:
            try:
                _toggle_resp = cls._toggle_proxy(SetBoolRequest(data = False))
                _toggle_resp_f = True
            except:
                rospy.logerr("Toggle unavailable")
        while cls.checkTurn() and not rospy.is_shutdown(): 
            diff =  (cls.getRadNorm(cls.last_target[2]) - cls.getRadNorm(cls.robot_pos[2]))
            if diff >= 3.1415:
               diff = -(6.283 - diff)
            elif diff < -3.1415:
                diff = 6.283 + diff
            coeff = cls.turn_coeff * abs(diff)
            if coeff > 1:
                coeff = 1
            elif coeff < cls.min_turn_coeff:
                coeff = cls.min_turn_coeff
            turn = diff/abs(diff) *  coeff      
            cls.cmdVel([0,0,turn])
            rospy.sleep(1/cls.update_rate)
        if _toggle_resp_f and _toggle_resp.message == "was 1":
            try:
                cls._toggle_proxy(SetBoolRequest(data = True))
            except:
                rospy.logerr("Toggle unavailable")
    @classmethod
    def fetchPoint(cls,current_pos):
        #current = cls.robot_pos 
        if cls.skipped >= cls.skip_thresh:
            rospy.loginfo(f"Goal failed! Sending Stop!")
            cls.status_publisher.publish(String('fail'))
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
                cls.status_publisher.publish(String('warn'))
                if cls.debug:
                    rospy.loginfo(f"Point failed cost check({point_cost})! Recursing...")
                cls.skipped += 1
                #cls.current_target += 1
                #cls.fetchPoint(current, target)
                return cls.fetchPoint(current_pos)
        if cls.debug:
            rospy.loginfo(f"Fetching target point = {point}\ncurr = {current_pos}")
        cls.skipped = 0 
        cls.current_target += 1
        return point 
       
    ####################################################################       
    # @classmethod
    # def updatePos(cls):
    #     Local.robot_pos = Local.new_pos
    @classmethod
    def checkPos(cls):
        return np.linalg.norm(cls.robot_pos[:2]-cls.actual_target[:2]) > cls.threshhold
    @classmethod
    def getCostCoeff(cls, cost: float):
        _cost_coeff = ((cost* 3)/cls.cost_threshhold)  * cls.cost_speed_coeff
        if _cost_coeff > cls.max_cost_speed_coeff:
            _cost_coeff = cls.max_cost_speed_coeff
        elif _cost_coeff < 1:
            _cost_coeff = 1
        return _cost_coeff
    @classmethod
    def updateTarget(cls):
        #rospy.logwarn(f"{cls.current_target =  }|{cls.robot_twist = }")
        cls.status_publisher.publish(String('ok'))
        if Local.debug:
            rospy.loginfo(f"robot pos {cls.robot_pos}")
        if cls.debug:
            rospy.loginfo(f'Updating target {cls.current_target},\n current = {cls.robot_pos} (max targs = {len(cls.targets)})')
        if cls.current_target < len(cls.targets)-1:
            cls.actual_target =  cls.fetchPoint(cls.robot_pos)
        elif cls.current_target == len(cls.targets)-1:
            cls.actual_target = cls.targets[-1]
        else:
            rospy.logerr(f"Current target overflow!")
            Local.goal_reached = 1
            shutdownHook()
        if cls.debug:
            rospy.loginfo(f'Riding to {cls.actual_target}')
        #rospy.loginfo(f"Pubbing marker {cls.actual_target[:2]}")
        
        while cls.checkPos() and not rospy.is_shutdown() and not cls.goal_reached:
            #Local.updatePos()
            speed_coeff = 1
            if cls.cost_coeff_enable:
                _cost_coeff = cls.getCostCoeff(cls.getCost(cls.actual_target))
                speed_coeff = speed_coeff / _cost_coeff
                rospy.logerr_once(f"Cost coeff = {_cost_coeff}")
                if speed_coeff > 1:
                    speed_coeff = 1
                elif speed_coeff < cls.min_coeff:
                    speed_coeff = cls.min_coeff
            if cls.path_coeff_enable:
                speed_coeff = speed_coeff * cls.getPathSpdCoeff()
            if speed_coeff > 1:
                speed_coeff = 1
            elif speed_coeff < cls.min_coeff:
                speed_coeff = cls.min_coeff
            cmd_target = cls.remapToLocal(cls.actual_target-cls.robot_pos-(cls.robot_twist*cls.inertia_compensation_coeff)) ###ADJUSTS GLOBAL COMAND TO LOCAL
            cls.cmdVel(cmd_target, speed_coeff*cls.static_coeff)#make slower at last point
            rospy.sleep(1/cls.update_rate)
        return
    ####################################################
# import os
# import cv2
def main():
    rospy.on_shutdown(shutdownHook)
    rate = rospy.Rate(Local.update_rate)
    while not rospy.is_shutdown():
            #_cost = Local.getCost(Local.actual_target)
            #rospy.logwarn(f'LOCAL:\n##### Cost of current robot pos is {_cost} (max is {Local.cost_threshhold})\n##### speed reduction = {Local.getCostCoeff(_cost)}')
        if not Local.goal_reached:
            Local.updateTarget()
            if np.linalg.norm(Local.robot_pos[:2] - Local.targets[-1][:2]) < Local.threshhold:
                if Local.rotate_at_end:
                    Local.rotateAtEnd()
                rospy.loginfo(f'Goal reached!')
                Local.status_publisher.publish(String('done'))
                Local.goal_reached = 1
                shutdownHook()
                #Local.current_target = len(Local.targets)-1
        rate.sleep()

    # os.chdir("~/catkin_ws/src/ebobot/config/costmap")
    # cv2.imwrite("local_recieved_map.png", Local.costmap)
    # rospy.sleep(5)

if __name__=="__main__":
    main()
