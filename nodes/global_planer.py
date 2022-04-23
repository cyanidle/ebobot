#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
#from cmath import sin,cos
from math import radians, ceil, sin, cos, atan
import tf
import numpy as np
#from threading import Thread
#Messages and actions
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped#, Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String, Float32
from ebobot.srv import ChangeCost, ChangeCostResponse
#from visualization_msgs.msg import Marker
######
#from dorlib import dCoordsOnCircle
import markers
######
#Пусть глобал планер посылает экшоны (Global nav_msgs/Path) в сторону локального и получает некий фидбек по выполнению, в случае ступора он вызвоет либо отдельный скрипт, либо просто некую функцию
#Внутри самого глобал планера, которая временно подтасует текущую цель на "ложную" которая позволит выехать из затруднения (Recovery Behavior)
#В остальное время планеру в тупую следуют указаниям скрипта поведения, посылающего команды в /simple_goal
####
import actionlib
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
#
####
rospy.init_node('global_planer')





################



#TO DO: Add experimental "make shortcuts" feature - if dist doesnt grow fast enough - remove intermediate points


##############
import cv2
import os
def robotPosCallback(odom):
    euler = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])
    Global.robot_pos = np.array([odom.pose.pose.position.y/Global.costmap_resolution,    odom.pose.pose.position.x/Global.costmap_resolution,    euler[2]%(3.1415*2)]) 
    Global.robot_twist = Global.twist_amplify_coeff * (np.array([odom.twist.twist.linear.y,
    odom.twist.twist.linear.x,
    odom.twist.twist.angular.z]) /
    Global.costmap_resolution
    )
def localStatusCallback(status):
    if MoveServer.use_actionlib:
        if move_server.server.is_active():
            move_server.update(status.data,local=1)
    else:
        move_server.update(status.data,local=1)
def targetCallback(target): 
    euler = tf.transformations.euler_from_quaternion([target.pose.orientation.x,target.pose.orientation.y,target.pose.orientation.z,target.pose.orientation.w])
    goal = [target.pose.position.y/Global.costmap_resolution,target.pose.position.x/Global.costmap_resolution,euler[2]%(3.1415*2)]
    rospy.loginfo(f"\n####################################################\nGOT NEW TARGET: {goal}\n####################################################\nStart from {Global.robot_pos}\n####################################################")
    Global.list.clear()
    Global.goal_reached = 0
    Global.target = np.array(goal)
    Global.list.append((np.array(Global.robot_pos[:2]),0)) #Здесь нужно получить по ебалу от негров!
    Global.start_pos = Global.robot_pos - Global.robot_twist
    Global.consecutive_jumps = 0
    Global.target_set = 1
    Global._fail_count = 0
    if Global.rviz_enable:
        #markers.pubMarker((target.pose.position.y,target.pose.position.x),0,add = 0,frame_name='global_target',debug=Global.debug)
        markers.pubMarker((target.pose.position.y,target.pose.position.x),0,add = 1,frame_name='global_target',debug=0)
    #!!!!!!!!!!!!!!
def costmapCallback(costmap):
    Global.costmap_resolution = costmap.info.resolution
    Global.costmap_height = costmap.info.height
    Global.costmap_width= costmap.info.width
    rospy.loginfo_once(f"Got new map, height = {Global.costmap_height}, width= {Global.costmap_width}")
    Global.costmap= np.reshape(costmap.data,(Global.costmap_height, Global.costmap_width))  
    Global.debug_map = Global.costmap
    
def costmapUpdateCallback(update): #not used
    rospy.loginfo("Got new map update") 
    origin_x = update.x
    origin_y = update.y
    for y in range (update.height):
        for x in range (update.width):
            Global.costmap[origin_y + y][origin_x + y] = update.data[x+y]

    
def changeCostCB(req):
    _was=Global._default_max_cost
    Global.change_cost_publisher.publish(Float32(_was))
    Global.change_cost_publisher.publish(Float32(req.cost))
    Global.maximum_cost = req.cost
    Global._default_max_cost = req.cost
    rospy.logwarn(f"GLOBAL PLANER: Cost changed to {req.cost}")
    return ChangeCostResponse(_was)
class Global(): ##Полная жопа
    step = rospy.get_param('~step',2) #in сells (with resolution 2x2 step of 1 = 2cm)
    #Params
    #Features
    rviz_enable = rospy.get_param('~rviz_enable',1)
    cleanup_feature = rospy.get_param('~cleanup_feature',1)
    experimental_cleanup_enable = rospy.get_param('~experimental_cleanup_feature',1)
    stuck_check_feature = rospy.get_param('~stuck_check_feature',1)
    debug = rospy.get_param('~debug',0)
    #/Features    
    #
    resend = rospy.get_param('~resend', 1)
    update_stop_thresh = rospy.get_param('~update_stop_thresh', 4) #in steps
    update_stop_thresh *= step
    update_rate = rospy.get_param('~update_rate',1) #per second
    sleep_on_fail_time = rospy.get_param('~sleep_on_fail_time',0.3)
    #
    accelerate_coeff = rospy.get_param('~accelerate_coeff',0.00022) #DO NOT TOUCH, shit goes haywire
    if experimental_cleanup_enable:
        accelerate_coeff = 0
    costmap_resolution = rospy.get_param('~costmap_resolution',0.02)   #cm/cell (default)
    #
    maximum_cost = rospy.get_param('~maximum_cost',40) 
    abs_max_cost = rospy.get_param("~absolute_max_cost", 90)
    _default_abs_max = abs_max_cost
    _default_max_cost = maximum_cost
    recovery_cost_step = rospy.get_param('~recovery_cost_step',1)
    recovery_cost_step /= update_rate
    #
    stuck_check_jumps = rospy.get_param('~jumps_till_stuck_check',15)
    stuck_dist_threshhold = rospy.get_param('~stuck_dist_threshhold',0.5) #in cells (if havent moved in the last (stuck check jumps))
    stuck_dist_threshhold *= step # * 2
    #
    twist_amplify_coeff = rospy.get_param('~twist_amplify_coeff',0.02)
    #
    dead_end_dist_diff_threshhold = rospy.get_param('~dead_end_dist_diff_threshhold',2) #in cells
    dead_end_dist_diff_threshhold *= step * 2
    maximum_jumps = rospy.get_param('~maximum_jumps',600)
    consecutive_jumps_threshhold = rospy.get_param('~consecutive_jumps_threshhold',5)
    fail_count_threshhold = rospy.get_param('~fail_count_threshhold',5)
    num_of_tries_for_last = rospy.get_param('~num_of_tries_for_last',5)
    dist_to_target_threshhold =  rospy.get_param('~global_dist_to_target_threshhold',3) #in cells
    dist_to_target_threshhold += step
    step_radians_resolution = rospy.get_param('~step_radians_resolution', 24)  #number of points on circle to try (even)
    #Cleanup params
    cleanup_repeats_len = rospy.get_param('~cleanup_repeats_len',8) #jumps (if doenst exeed thr in (len) jumps - deleted)
    cleanup_power = rospy.get_param('~cleanup_power',1) #num times cleaning is used (1 is best, 2 for open spaces)   
    cleanup_repeats_threshhold = rospy.get_param('~cleanup_repeats_threshhold',step * cleanup_repeats_len * cleanup_power * 0.5 ) #cells CHANGE CAREFULLY!!
    cleanup_repeats_threshhold = cleanup_repeats_threshhold * step * cleanup_repeats_len
    #/Params
    

    #Topics
    local_status_topic = rospy.get_param('~local_status_topic', 'planers/local/status')
    rviz_point_topic = rospy.get_param('~rviz_point_topic', 'global_points')
    rviz_topic = rospy.get_param('costmap_server/rviz_topic','rviz_path')
    costmap_topic = rospy.get_param('~costmap_topic','costmap_server/costmap')
    costmap_update_topic = rospy.get_param('~costmap_update_topic','costmap_server/updates')
    path_publish_topic =  rospy.get_param('~path_publish_topic', 'planers/global/path')
    pose_subscribe_topic =  rospy.get_param('~pose_subscribe_topic', 'move_base_simple/goal')
    change_cost_service_name = rospy.get_param('/global/change_cost_service_name', 'change_cost_service')
    robot_pos_topic =  rospy.get_param('~robot_pos_topic',"odom")
    ####
    path_broadcaster = tf.TransformBroadcaster()
    rospy.Service(change_cost_service_name,ChangeCost,changeCostCB)
    change_cost_publisher = rospy.Publisher(f"{change_cost_service_name}_echo", Float32, queue_size=10)
    local_status_subscriber = rospy.Subscriber(local_status_topic, String, localStatusCallback)
    costmap_update_subscriber = rospy.Subscriber(costmap_update_topic, OccupancyGridUpdate, costmapUpdateCallback)
    costmap_subscriber = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback)
    robot_pos_subscriber = rospy.Subscriber(robot_pos_topic, Odometry, robotPosCallback)
    target_subscriber = rospy.Subscriber(pose_subscribe_topic, PoseStamped, targetCallback)
    path_publisher = rospy.Publisher(path_publish_topic, Path, queue_size=10)
    rviz_publisher = rospy.Publisher(rviz_topic, Path, queue_size=5)
    ###
    rospy.loginfo("Topics init")
    #/Topics
    ################################################ global values
    _last_change = 0
    _return_local_cost_flag = 0
    _fail_count = 0
    robot_twist = np.array([0,0,0])
    target_set = 0
    last_stuck = np.array([0,0])
    debug_map = []
    error = 0
    goal_reached = 1
    start_pos = np.array([0,0,0])
    robot_pos = np.array([0,0,0])
    list = []
    default_costmap_list = [[0]*101 for _ in range(151)]
    costmap = np.array(default_costmap_list) 
    #costmap_resolution = 0.02 #meters per cell (default param comes from costmap server)
    costmap_width= 151
    costmap_height = 101
    target = np.array([0,0,0])
    num_jumps = 0
    consecutive_jumps = 0
    #####################################
    lock_dir = False
    lock_dirs = rospy.get_param( "~lock_dirs", 
    [0,  'left','left', 'right',  'right', 'top', 'bot']) #these directions decide in which order robot tries different lock directions
    lock_dir_num = 0
    #Debug
    if debug:
        rospy.loginfo(f"costmap = {costmap}")
    #/Debug
    #################### Rotors list
    rotors_list = []
    @staticmethod
    def initRotors():
        for num in range(Global.step_radians_resolution//2 * 2):
            step = radians(360)/Global.step_radians_resolution
            if num%2:
                dir = 1
            else:
                dir = -1
            num = num//2 
            rotor = cos(dir * step * num) + 1j * sin(dir * step * num)
            Global.rotors_list.append(rotor)
    @classmethod
    def appendToList(cls,pos,dist):
        cls.list.append((pos,dist))
        if cls.debug:
            cls.debug_map[int(pos[0])][int(pos[1])] = 255
            rospy.loginfo(f"appended {cls.list[-1]}")
    #########################
    @classmethod
    def dirGenerator(cls,delta_vect,num):
        turn = cls.rotors_list[num]
        norm = np.linalg.norm(delta_vect)
        normalized = delta_vect/norm
        imag = normalized[0] + 1j * normalized[1]
        if cls.debug:
            rospy.loginfo(f"Generator called, turn, norm, normalized,imag = {turn},{norm},{normalized},{imag}")
        next_normalized = imag * turn
        next_pos = next_normalized * norm
        if cls.debug:
            rospy.loginfo(f"Yielding x {next_pos.real}, y {next_pos.imag}")
        yield (next_pos.real,next_pos.imag) #needs testing
    def checkIfStuck(num):
        if len(Global.list) < 3:
            return
        if not (num-Global.stuck_check_jumps)%Global.stuck_check_jumps:
            if Global.last_stuck.any() and num and np.linalg.norm(Global.last_stuck - Global.list[-1][0][:2]) < Global.stuck_dist_threshhold:
                #rospy.logwarn(f"Planer stuck, using recovery!")
                Global.lock_dir_num += 1
                Global.lock_dir_num  = Global.lock_dir_num%len(Global.lock_dirs)
            else:
                Global.lock_dir_num = 0
            Global.last_stuck = Global.list[-1][0][:2]
            Global.lock_dir = Global.lock_dirs[Global.lock_dir_num]
    @staticmethod
    def reset(): #For reset service
        Global.list.clear()
        Global.list.append((np.array(Global.robot_pos[:2]),0)) #Здесь нужно получить по ебалу от негров!
        Global.start_pos = Global.robot_pos - Global.robot_twist
        Global.consecutive_jumps = 0
        Global.target_set = 1
        Global.num_jumps = 0
    #################################################Main bullshit
    @staticmethod
    def appendNextPos(): #uses only x and y, the needed orientations should be set after the goal list is complete
        current_pos = Global.list.pop()[0][:2]  #this is the last and current position
        if Global.debug:
            rospy.loginfo(f"###############################\nPopped {current_pos}") 
        target_vect = np.array(Global.target[:2]) - current_pos
        _d_norm = np.linalg.norm(target_vect)
        if not _d_norm:
            delta_vect = np.array(0,0,target_vect[2])
        else:
            delta_vect = target_vect/_d_norm * Global.step
        next_pos = current_pos + delta_vect 
        Global.num_jumps += 1
        if Global.debug:
            rospy.loginfo(f"\nnext_pos = {next_pos}\ncurrent_pos = {current_pos}\ntarget_vect = {target_vect}\ndelta_vect = {delta_vect}")
            rospy.loginfo(f"Append called, num of jumps = {Global.num_jumps}")
        for num in range(len(Global.rotors_list)):
            for coords in Global.dirGenerator(delta_vect,num):
                y,x = coords
                next_pos_y,next_pos_x = round(float(next_pos[0])),round(float(next_pos[1])) #updated later
                ##################################
                if Global.lock_dir:
                    if Global.lock_dir == 'top':
                        y = abs(y)
                    elif Global.lock_dir == 'right':
                        x = abs(x)
                    elif Global.lock_dir == 'left':
                        x = -abs(x)
                    elif Global.lock_dir == 'bot':
                        y = -abs(y)
                ####################################################
                if Global.debug:
                    #rospy.loginfo(f"next_pos_x = {next_pos_x},next_pos_y = {next_pos_y}")
                    rospy.loginfo(f"num = {num}, cons_jumps = {Global.consecutive_jumps} ")
                    pass
                ####################################################
                current_dist = np.linalg.norm(current_pos - Global.start_pos[:2]) #get current dist to starting point
                next_dist = np.linalg.norm(next_pos - Global.start_pos[:2])       #and for next point
                ######################################################
                if Global.num_jumps > Global.maximum_jumps:                       #check for jumps overflow
                    rospy.logerr(f"Jumps > maximum({Global.maximum_jumps}), fail score {Global._fail_count}")
                    Global.costRecover()
                    Global.goal_reached = 1
                    Global.error = 0
                    Global.num_jumps = 0
                    Global._fail_count += Global.fail_count_threshhold/Global.num_of_tries_for_last
                    rospy.sleep(Global.sleep_on_fail_time)
                    Global.checkFail()
                    return
                ##################################################
                if next_pos_x >= Global.costmap_width or next_pos_x < 0:
                    continue
                if next_pos_y >=  Global.costmap_height or next_pos_y < 0 :
                    continue
                else:
                    if Global.debug:
                        rospy.loginfo(f"next cost = {Global.costmap[next_pos_y][next_pos_x]} ")
                ##################################################
                if Global.num_jumps == 1:
                    Global.appendToList(current_pos,0)
                
                if Global.costmap[next_pos_y][next_pos_x] < Global.maximum_cost:                          #if next cell is acceptable then             
                    if np.linalg.norm(Global.target[:2] - current_pos) < Global.dist_to_target_threshhold:#its checked to be close enough to the goal
                        Global.num_jumps = 0                                                              #if too close - ignored, last target added
                        Global.goal_reached = 1
                        #
                        if Global.maximum_cost != Global._default_max_cost:
                            Global._return_local_cost_flag = 1
                            Global.maximum_cost = Global._default_max_cost #For recovery
                        Global._fail_count = 0
                        #
                        if not Global.resend:
                            Global.target_set = 0
                        Global.appendToList(Global.target,np.linalg.norm(Global.target - Global.start_pos))
                        return
                    elif Global.consecutive_jumps > Global.consecutive_jumps_threshhold:                  #additional point is added in case of long straight jump
                        Global.appendToList(current_pos,current_dist)
                        Global.appendToList(next_pos,next_dist)
                        Global.consecutive_jumps = 0                                                      #consecutive jumps counter reset
                        return
                    elif num > 1:      
                        Global.appendToList(current_pos,current_dist)                                                                                 
                        Global.appendToList(next_pos,next_dist)   #if a turn is made, the laast point is also saved
                        Global.consecutive_jumps += 1
                        return
                    else:    
                        Global.consecutive_jumps += 1                                #otherwise remove current, add next
                        Global.appendToList(next_pos,next_dist)
                        return
                else:
                    if Global.stuck_check_feature:
                        Global.checkIfStuck(Global.num_jumps)
                    next_pos = current_pos + np.array([y,x])
                    next_pos = next_pos + next_pos*Global.accelerate_coeff*Global.num_jumps
                    if Global.debug:
                        rospy.logwarn(f"Position failed (cost)!")
        if len(Global.list) == 0:
            rospy.logerr(f"All start points failed! Goal ignored| Current cost step {Global.recovery_cost_step}")
            Global.costRecover()
            Global.list.clear()
            Global.list.append((np.array(Global.robot_pos[:2]),0)) #Здесь нужно получить по ебалу от негров!
            Global.start_pos = Global.robot_pos - Global.robot_twist
            Global.target_set = 1
            ####
            #Global.error = 1 
            Global.checkFail()
            #if not Global._fail_count%200:
            #   rospy.logwarn("GLOBAL: failing cost checks (stuck at beginning)!")
        else:
            Global.checkFail()
            #if not Global._fail_count%200:
            #   rospy.logwarn("GLOBAL: failing cost checks!")
    @classmethod
    def costRecover(cls):
        cls.change_cost_publisher.publish(Float32(cls.maximum_cost))
        cls.maximum_cost += cls.recovery_cost_step
        cls.change_cost_publisher.publish(Float32(cls.maximum_cost))
    @classmethod
    def costDefault(cls):
        Global.change_cost_publisher.publish(Float32(Global.maximum_cost))
        cls.maximum_cost = cls._default_max_cost
        Global.change_cost_publisher.publish(Float32(Global.maximum_cost)) 
    @classmethod
    def checkFail(cls):
        cls._fail_count += 1
        if cls._fail_count >= cls.fail_count_threshhold:
            rospy.logerr(f"Global planer cancels current goal!! Thresh {cls.fail_count_threshhold}| current {cls._fail_count}")
            cls.costDefault()
            if cls.resend:
                cls.target_set = 0
                cls.goal_reached = 1
            else:
                cls.goal_reached = 1
            cls.list = []
            cls.publish(empty = True)
            move_server.done(0)
            cls._fail_count = 0
            cls.error = 1
            if Global.maximum_cost != Global._default_max_cost:
                Global._return_local_cost_flag = 1
                Global.maximum_cost = Global._default_max_cost #For recovery
    @staticmethod 
    def cleanupDeadEnds():
        list_to_remove = []
        max_dist_num = 0
        _jumps = int(round(Global.update_stop_thresh/Global.step))
        if len(Global.list) < _jumps * 2:
            return
        for num, _tuple in enumerate(Global.list[1:-_jumps]):
            num += 1
            pos, dist = _tuple
            curr_dist = dist-Global.list[num-2][1]
            if max_dist_num:
                if num != max_dist_num:
                    curr_dist = np.linalg.norm(pos[:2]-Global.list[num-2][0][:2]) 
                else:
                    curr_dist = 100
            if curr_dist > Global.dead_end_dist_diff_threshhold:
                continue
            else:
                if 3 < num < (len(Global.list)-_jumps):
                    list_to_remove.append(num-2)
                    if np.linalg.norm(Global.list[num+2][0][:2] -  pos[:2])> Global.dead_end_dist_diff_threshhold:
                        max_dist_num = num+2
        for done,num in enumerate(list_to_remove):
            if (num-done) in range(len(Global.list)):
                popped = Global.list.pop(num-done)
    @staticmethod
    def cleanupRepeats():
        list_to_remove = []
        ignore_list = []
        max_num = len(Global.list)-1
        check_for = Global.cleanup_repeats_len
        for num in range(len(Global.list)):
            if num > check_for and (max_num-num) > check_for:
                if not num-check_for in range(len(Global.list)):
                    continue
                if np.linalg.norm(Global.list[num][0][:2] - Global.list[num - check_for][0][:2]) < Global.cleanup_repeats_threshhold:
                    for i in range(num - check_for,num):
                        if not i in list_to_remove:
                            list_to_remove.append(i)
                            ignore_list.append(num)
        for done,num in enumerate(list_to_remove):
            if not num in ignore_list:
                popped = Global.list.pop(num-done)
                if Global.debug:
                    rospy.loginfo(f"Removing repeats {popped}")
    ###########################################3
    @staticmethod
    def sendTransfrom(y , x, th):      
        Global.path_broadcaster.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0,0,0),
            rospy.Time.now(),
            "costmap",
            "rviz_path"
            )
    @staticmethod
    def publish(*,empty = False):
        ##
        rviz = Path()
        rviz.header.frame_id = "/costmap"
        rviz.header.stamp = rospy.Time.now()
        ###
        rviz_coeff = (1/Global.costmap_resolution) #DIVIDE BY IT
        msg = Path()
        msg.header.frame_id = "/costmap" ###????????
        msg.header.stamp = rospy.Time.now()
        if len(Global.list):
            target = Global.list.pop()[0]
            for goal,_ in Global.list:
                    pose = PoseStamped()
                    pose.pose.position.y = goal[0]
                    pose.pose.position.x = goal[1]
                    if Global.debug:
                        rospy.loginfo(f"New point {goal}") 
                    msg.poses.append(pose)
                    if Global.rviz_enable:
                        r = PoseStamped()
                        r.pose.position.y = goal[0] / rviz_coeff
                        r.pose.position.x = goal[1] / rviz_coeff
                        rviz.poses.append(r)
            target_pos = PoseStamped()
            if Global.debug:
                rospy.loginfo(f"Last point {target}")
            if target.size<3:
                target = np.append(target,0)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, target[2])
            if Global.rviz_enable:
                rviz_targ = PoseStamped()
                rviz_targ.pose.position.y = target[0] / rviz_coeff
                rviz_targ.pose.position.x = target[1]/rviz_coeff
                if target.size < 3:
                    target = np.append(target, 0)
                rviz_quat = tf.transformations.quaternion_from_euler(0, 0, target[2]/ (1/Global.costmap_resolution))
                rviz_targ.pose.orientation.x = rviz_quat[0]
                rviz_targ.pose.orientation.y = rviz_quat[1]
                rviz_targ.pose.orientation.z = rviz_quat[2]
                rviz_targ.pose.orientation.w = rviz_quat[3]
                rviz.poses.append(rviz_targ)
                Global.sendTransfrom(target[0]/ rviz_coeff,target[1]/ rviz_coeff,target[2])
                Global.rviz_publisher.publish(rviz)
            target_pos.pose.orientation.x = quaternion[0]
            target_pos.pose.orientation.y = quaternion[1]
            target_pos.pose.orientation.z = quaternion[2]
            target_pos.pose.orientation.w = quaternion[3]
            target_pos.pose.position.y = target[0]
            target_pos.pose.position.x = target[1]
            if Global.debug:
                rospy.loginfo(f"Last point {target}") 
            msg.poses.append(target_pos)
            if empty:
                Global.path_publisher.publish(Path())
            else:
                Global.path_publisher.publish(msg)    
def shutdownHook():
    Global.list = [(Global.robot_pos,0)]
    Global.publish()




def main():
    Global.initRotors()
    rospy.on_shutdown(shutdownHook)
    while not rospy.is_shutdown():
        if Global.resend:
            Global.goal_reached = 0
            Global.list.clear()
            Global.list.append((np.array(Global.robot_pos[:2]),0))
            Global.start_pos = Global.robot_pos+Global.robot_twist #Здесь нужно получить по ебалу от негров!s
            Global.consecutive_jumps = 0
        if Global.target_set:
            while not Global.goal_reached and not rospy.is_shutdown():
                Global.appendNextPos()
            if Global.maximum_cost > Global.abs_max_cost:
                rospy.logerr("GLOBAL: Maximum cost is bigger than threshhold!")
                Global._fail_count = Global.fail_count_threshhold
                Global.checkFail()
                Global.costDefault()
            if Global.cleanup_feature:
                for _ in range(Global.cleanup_power):
                    Global.cleanupDeadEnds()
                    if Global.experimental_cleanup_enable:
                        Global.cleanupRepeats()
            if len(Global.list) and not Global.error:
                Global.publish()
            Global.error = 0
            if Global.resend:
                if np.linalg.norm(Global.robot_pos[:2] - Global.target[:2]) < Global.update_stop_thresh:
                    Global.target_set = 0
                    Global._fail_count = 0         
        rate.sleep()




###########################################################
from ebobot.srv import SetMoveTarget, SetMoveTargetResponse 
def SetMoveCB(goal):
    ###################
    if move_server.active:
        move_server._preemted = 1
        rospy.logerr(f"PIZDEC, YA YEDU NA DRUGOI HUI({goal.x, goal.y})")
    else:
        move_server.active = 1
        rospy.logerr(f"PIZDEC, YA YEDU NAHUI ({goal.x, goal.y})")
    ###################
    new_target = PoseStamped()
    new_target.pose.position.x = goal.x
    new_target.pose.position.y = goal.y
    quat = tf.transformations.quaternion_from_euler(0,0,goal.theta)
    new_target.pose.orientation.x = quat[0]
    new_target.pose.orientation.y = quat[1]
    new_target.pose.orientation.z = quat[2]
    new_target.pose.orientation.w = quat[3]
    #targetCallback(new_target)
    move_server.reset()
    move_server.execute()
    targetCallback(new_target)
    resp = SetMoveTargetResponse()
    resp.preempted= bool(move_server._preempted)
    resp.status = move_server.feedback
    rospy.loginfo(f"GLOBAL: Service responce(preemted = {resp.preempted}, status = {resp.status})")
    return resp
#############################################################    

class MoveServer:
    use_actionlib = rospy.get_param("/global/use_actionlib", 1)
    _preemted = 0
    if not use_actionlib:
        service_name = rospy.get_param("/global/move_service_name", "set_move_service")
        rospy.Service(service_name, SetMoveTarget,SetMoveCB)
        feedback = "good"
        rospy.logwarn("GLOBAL: WARNING - USING EXPERIMENTAL COMMANDS (NOT ACTIONLIB)")
        def __init__(self) -> None:
            self.active = 0
            self._preempted = 0
            self._success_flag = 0
            self._fail_flag = 0 
            self.feedback = type(self).feedback
        def reset(self):
            self.active = 0
            self._success_flag = 0
            self._fail_flag = 0 
        def execute(self):
            self.feedback = "executing"
            self.active = 1
            self._success_flag = 0
            self._fail_flag = 0 
            while (not self._fail_flag and not rospy.is_shutdown() and not self._success_flag
            and not self._preemted):
                rospy.sleep(0.1)
            rospy.logwarn(f"GLOBAL: Exiting service loop!\n fail: {self._fail_flag}, success: {self._success_flag}, preempt: {self._preempted}, rospy {rospy.is_shutdown()}")
            if self._success_flag:
                self.feedback = "done"
            else:
                self.feedback = "fail"
            self.active = 0
            self._success_flag = 0
            self._fail_flag = 0 
    else:
        feedback = MoveFeedback('good')
        def __init__(self):
            self.server = actionlib.SimpleActionServer('move', MoveAction, self.execute, False)
            self.server.start()
            self._success_flag = 0
            self._fail_flag = 0
        def execute(self,goal):
            #MoveServer._preemted  = 1
            rospy.logerr(f"PIZDEC, YA YEDU NAHUI ({goal.x, goal.y})")
            ###################
            new_target = PoseStamped()
            new_target.pose.position.x = goal.x
            new_target.pose.position.y = goal.y
            quat = tf.transformations.quaternion_from_euler(0,0,goal.theta)
            new_target.pose.orientation.x = quat[0]
            new_target.pose.orientation.y = quat[1]
            new_target.pose.orientation.z = quat[2]
            new_target.pose.orientation.w = quat[3]
            targetCallback(new_target)
            ###################
            #rospy.logerr(f"Global {self._success_flag = }|{self._fail_flag = }")
            #
            while (not rospy.is_shutdown() and self._success_flag == 0 
            and self._fail_flag == 0):
                #rospy.logwarn(f"Robot driving to target")
                rospy.sleep(0.05)
            if self._success_flag:
                self.server.set_succeeded(MoveResult(0))
            elif self._fail_flag:
                self.server.set_aborted(MoveResult(1))
            #else:
            #    self.server.set_preempted(MoveResult(4))
            self._fail_flag, self._success_flag = 0, 0#, 0
            #self.feedback = "init"
    def update(self, fb, local=0):
        self.feedback = fb
        if local:
            if MoveServer.use_actionlib:
                self.server.publish_feedback(MoveFeedback(self.feedback))
            if self.feedback == "fail":
                self.done(0)
            elif self.feedback == "done":
                self.done(1)
        elif MoveServer.use_actionlib:
            self.server.publish_feedback(MoveFeedback(self.feedback))
    def done(self,status:int):
        "Status 1 = success, status 0 = fail"
        Global._fail_count = 0
        if Global._return_local_cost_flag:
            Global.change_cost_publisher.publish(Float32(Global.maximum_cost))
            Global._return_local_cost_flag = 0
        if status:
            self._success_flag = 1       
        else:
            self._fail_flag  = 1
            Global.target_set = 0
            Global.goal_reached = 1
#####################################################
if __name__=="__main__":
    move_server = MoveServer()
    rate = rospy.Rate(Global.update_rate)
    main()
