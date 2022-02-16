#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
#import math
import tf
import numpy as np
rospy.init_node('global_planer')
#Messages and actions
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Path, OccupancyGrid, Odometry
######
from dorlib import dCoordsOnCircle
######
#Пусть глобал планер посылает экшоны (Global nav_msgs/Path) в сторону локального и получает некий фидбек по выполнению, в случае ступора он вызвоет либо отдельный скрипт, либо просто некую функцию
#Внутри самого глобал планера, которая временно подтасует текущую цель на "ложную" которая позволит выехать из затруднения (Recovery Behavior)
#В остальное время планеру в тупую следуют указаниям скрипта поведения, посылающего команды в /simple_Global


def robotPosCallback(odom):
    Global.robot_pos = np.array([odom.pose.position.x,odom.pose.position.y,tf.transformations.euler_from_quarternion(odom.pose.orientation)[2]])

def targetCallback(target): 
    euler = tf.transformations.euler_from_quaternion([target.pose.orientation.x,target.pose.orientation.y,target.pose.orientation.z,target.pose.orientation.w])
    goal = [target.pose.position.x,target.pose.position.y,euler[2]]
    rospy.loginfo(f"\n####################################################\n GOT NEW TARGET: {goal}\n####################################################")
    Global.setNew(goal)
def costmapCallback(costmap):
    rospy.loginfo("Got new map")
    Global.costmap_resolution = costmap.info.resolution
    Global.costmap_width = costmap.info.width
    Global.costmap_height = costmap.info.height
    for y in range(costmap.info.width+1):
        for x in range(costmap.info.height+1):
            Global.costmap[x][y] = costmap.data[x+y]
    pass #Dodelai
def costmapUpdateCallback(update):
    rospy.loginfo("Got new map update")
    origin_x = update.x
    origin_y = update.y
    for x in range (update.height+1):
        for y in range (update.width+1):
            Global.costmap[origin_x + x][origin_y + y] = update.data[x+y]

    

class Global(): ##Полная жопа
    #Params
    costmap_update_topic = rospy.get_param('global_planer/costmap_update_topic','/costmap_update')
    costmap_topic = rospy.get_param('global_planer/costmap_topic','/costmap')
    maximum_cost = rospy.get_param('global_planer/maximum_cost',30)
    cleanup_feature = rospy.get_param('global_planer/cleanup_feature',1)
    stuck_check_feature = rospy.get_param('global_planer/stuck_check_feature',1)
    stuck_check_jumps = rospy.get_param('global_planer/jumps_till_stuck_check',10)
    stuck_dist_threshhold = rospy.get_param('global_planer/stuck_dist_threshhold ',5) #in cells (if havent move in the alast stuck check jumps)
    update_rate = rospy.get_param('global_planer/update_rate',2)
    dead_end_dist_diff_threshhold = rospy.get_param('global_planer/dead_end_dist_diff_threshhold',2) #in cells
    maximum_jumps = rospy.get_param('global_planer/maximum_jumps',400)
    consecutive_jumps_threshhold = rospy.get_param('global_planer/consecutive_jumps_threshhold',4)
    robot_pos_topic =  rospy.get_param('global_planer/robot_pos_topic',"/odom")
    debug = rospy.get_param('global_planer/debug',1)
    dist_to_target_threshhold =  rospy.get_param('global_planer/global_dist_to_target_threshhold',4) #in cells
    step = rospy.get_param('global_planer/step',4) #in cm, depends on cells (with resolution 2x2 step of 1 = 2cm)
    step_circle_resolution = rospy.get_param('global_planer/step_circle_resolution', 3)  #number of slices on circle circumference (more = more round)
    path_publish_topic =  rospy.get_param('global_planer/path_publish_topic', 'global_path')
    pose_subscribe_topic =  rospy.get_param('global_planer/pose_subscribe_topic', 'target_pose')
    #/Params
    #Topics
    path_broadcaster = tf.TransformBroadcaster()
    costmap_update_subscriber = rospy.Subscriber(costmap_update_topic, OccupancyGridUpdate, costmapUpdateCallback)
    costmap_subscriber = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback)
    robot_pos_subscriber = rospy.Subscriber(robot_pos_topic, PoseStamped, robotPosCallback)
    target_subscriber = rospy.Subscriber(pose_subscribe_topic, PoseStamped, targetCallback)
    path_publisher = rospy.Publisher(path_publish_topic, Path, queue_size=10)
    ###
    rospy.loginfo("Topics init")
    #/Topics
    ################################################ global values
    goal_reached = 1
    start_pos = np.array([0,0,0])
    robot_pos = np.array([0,0,0])
    list = []
    default_costmap_list = []
    if debug:
        default_costmap_list = [[0]*101 for _ in range(151)]
        for x in range(50,80):
            for y in range (0,80):
                default_costmap_list[x][y] = 60
    costmap = np.array(default_costmap_list) 
    costmap_resolution = 2
    costmap_height = 151
    costmap_width = 101
    target = np.array([0,0,0])
    num_jumps = 0
    consecutive_jumps = 0
    poses = dCoordsOnCircle(step, step_circle_resolution)
    lock_dir = False
    lock_dirs = [0, 'right', 'left', 'top' , 'bot']
    lock_dir_num = 0
    #Debug
    if debug:
        rospy.loginfo(f"costmap = {costmap}")
    #/Debug


    @staticmethod
    def setNew(new_Global = [0,0,0]): #new_Global is a list [x,y,th] where x and y are cells on costmap
        Global.list.clear()
        if Global.debug:
            rospy.loginfo(f"List after clearing {Global.list}")
        Global.goal_reached = 0
        Global.target = np.array(new_Global)
        #Global.costmap = [[]] #here we shoudl retrieve global_costmap from server
        #!!!!!!!!!!!!!!
        Global.list.append((Global.robot_pos[:2],0)) #Здесь нужно получить по ебалу от негров, потому объявление войны было ошибкой!
        Global.start_pos = Global.robot_pos
        Global.consecutive_jumps = 0
        #!!!!!!!!!!!!!!
    @staticmethod
    def checkIfStuck(num):
        if num%Global.stuck_check_jumps == 0:
            if np.linalg.norm(Global.list[-1][0] - Global.list[-1*Global.stuck_check_jumps][0]) < Global.stuck_dist_threshhold:
                Global.lock_dir_num += 1
            else:
                Global.lock_dir_num = 0
            Global.lock_dir = Global.lock_dirs[Global.lock_dir_num]
    @staticmethod
    def reset(): #For reset service
        blank = np.array([0,0,0])
        Global.target = blank
        #Global.robot_pos = blank
    @staticmethod
    def appendNextPos(): #uses only x and y, the needed orientations should be set after the goal list is complete
        current_pos = Global.list.pop()[0]  #this is the last and current position
        target_vect = np.array(Global.target[:2]) - current_pos
        delta_vect = target_vect/np.linalg.norm(target_vect) * Global.step
        next_pos = current_pos + delta_vect 
        Global.num_jumps += 1
        if Global.debug:
            rospy.loginfo(f"\nnext_pos = {next_pos}\ncurrent_pos = {current_pos}\ntarget_vect = {target_vect}")
            rospy.loginfo(f"Append called, num of jumps = {Global.num_jumps}")
            #rospy.loginfo(f"next_pos = {next_pos[0]}")
        for num,coords in enumerate(Global.poses):
            if Global.stuck_check_feature:
                Global.checkIfStuck(num)
            x,y = coords
            if Global.lock_dir == 'top':
                x = abs(x)
                #y = abs(y)
            elif Global.lock_dir == 'right':
                #x = -abs(x)
                y = abs(y)
            elif Global.lock_dir == 'left':
                #x = abs(x)
                y = -abs(y)
            elif Global.lock_dir == 'bot':
                x = -abs(x)
                #y = abs(y)
            next_pos_x,next_pos_y = round(float(next_pos[0])),round(float(next_pos[1]))
            if Global.debug:
                rospy.loginfo(f"next_pos_x = {next_pos_x},next_pos_y = {next_pos_y}")
                rospy.loginfo(f"num = {num}, cons_jumps = {Global.consecutive_jumps} ")
            current_dist = np.linalg.norm(next_pos - Global.start_pos[:2]) #get current dist to starting point
            if Global.num_jumps > Global.maximum_jumps:                                               #check for jumps overflow
                rospy.logerror(f"Jumps > maximum({Global.maximum_jumps})")
                Global.goal_reached = 1
                break
            if abs(next_pos_x) > Global.costmap_height:
                continue
            if abs(next_pos_y) >  Global.costmap_width:
                continue
            if Global.costmap[next_pos_x][next_pos_y] < Global.maximum_cost:   #if next cell is acceptable then
                #rospy.loginfo(f"{Global.target[:2]}")        
                #rospy.loginfo(f"{current_pos}")               
                if np.linalg.norm(Global.target[:2] - current_pos) < Global.dist_to_target_threshhold:    #its checked to be close enough to the goal
                    Global.num_jumps = 0                                                              #if too close - ignored, last target added
                    Global.goal_reached = 1
                    Global.list.append((Global.target,current_dist))
                    break
                elif Global.consecutive_jumps > Global.consecutive_jumps_threshhold:                  #additional point is added in case of long straight jump
                    Global.list.append((current_pos,np.linalg.norm(current_pos - Global.start_pos[:2])))  
                    Global.list.append((next_pos,current_dist))  
                    Global.consecutive_jumps = 0                                                      #consecutive jumps counter reset
                    break
                elif num > 1:                                                                                           
                    Global.list.append((next_pos,np.linalg.norm(next_pos - Global.start_pos[:2])))    #if a turn is made, the laast point is also saved
                    Global.consecutive_jumps += 1
                    break
                else:    
                    Global.consecutive_jumps += 1
                    #Global.list.append((current_pos,current_dist))                                    #otherwise remove current, add next
                    Global.list.append((next_pos,current_dist))
                    break
            else:
                next_pos = current_pos + np.array([x,y])
                if Global.debug:
                    rospy.loginfo(f"nextpos = {next_pos}")
    @staticmethod 
    def cleanupDeadEnds():
        list_to_remove = []
        max_dist = 0
        for num, tuple in enumerate(Global.list):
            _, dist = tuple
            if dist > max_dist:
                max_dist = dist
            else:
                for subnum , tuple in enumerate(Global.list[:num]):
                    point, subdist = tuple
                    if subdist-dist>Global.dead_end_dist_diff_threshhold:
                        rospy.loginfo(f"Scheduling point {point} for removal")
                        list_to_remove.append(subnum)
                for done,num in enumerate(list_to_remove):
                    rospy.loginfo(f"Removing {Global.list.pop(num-done}")
  
    @staticmethod
    def sendTransfrom(x , y, th):      
        Global.path_broadcaster.sendTransform(
            (x, y, 0),
            tf.transformations.quaternion_from_euler(0,0,th),
            rospy.Time.now(),
            "map",
            "path"
            )
    @staticmethod
    def publish():
        msg = Path()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        target = Global.list.pop()[0]
        for goal,_ in Global.list:
                pose = PoseStamped()
                pose.pose.position.x = goal[0]
                pose.pose.position.y = goal[1]
                Global.sendTransfrom(goal[0],goal[1],0)
                if Global.debug:
                    rospy.loginfo(f"New point {goal}") 
                msg.poses.append(pose) 
        target_pos = PoseStamped()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, target[2])
        target_pos.pose.orientation.x = quaternion[0]
        target_pos.pose.orientation.y = quaternion[1]
        target_pos.pose.orientation.z = quaternion[2]
        target_pos.pose.orientation.w = quaternion[3]
        target_pos.pose.position.x = target[0]
        target_pos.pose.position.y = target[1]
        Global.sendTransfrom(target[0],target[1],target[2])
        if Global.debug:
            rospy.loginfo(f"Last point {target}") 
        msg.poses.append(target_pos)
        Global.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Global.list)+1} points") 

if __name__=="__main__":
    rate = rospy.Rate(Global.update_rate)
    while not rospy.is_shutdown():

        if not Global.goal_reached:
            start_time = rospy.Time.now() ### start time
            while not Global.goal_reached:
                Global.appendNextPos()
                if Global.debug:
                    rospy.loginfo(f"appended {Global.list[-1]}")
            if Global.cleanup_feature:
                Global.cleanupDeadEnds()
                rospy.loginfo("Dead Ends cleaned up!")
            Global.publish()
            end_time = rospy.Time.now() ### end time
            rospy.loginfo(f"Route made in {(end_time - start_time).to_sec()} seconds")

        rate.sleep()
