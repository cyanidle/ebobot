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
    Global.robot_pos[0] = np.array([odom.pose.position.x,odom.pose.position.y,tf.transformations.euler_from_quarternion(odom.pose.orientation)[2]])

def targetCallback(target): 
    goal = [target.pose.position.x,target.pose.position.y,tf.transformations.euler_from_quarternion(target.pose.orientation)[2]]
    Global.setNew(goal)
def costmapCallback(costmap):
    Global.costmap_resolution = costmap.info.resolution
    for y in range(costmap.info.width+1):
        for x in range(costmap.info.height+1):
            Global.costmap[x][y] = costmap.data[x+y]
    pass #Dodelai
def costmapUpdateCallback(update):
    origin_x = update.x
    origin_y = update.y
    for x in range (update.height+1):
        for y in range (update.width+1):
            Global.costmap[origin_x + x][origin_y + y] = update.data[x+y]

    

class Global(): ##Полная жопа
    #Params
    costmap_update_topic = rospy.get_param('planers/costmap_update_topic','/costmap_update')
    costmap_topic = rospy.get_param('planers/costmap_topic','/costmap')
    maximum_cost = rospy.get_param('planers/maximum_cost',30)
    cleanup_feature = rospy.get_param('planers/cleanup_feature',1)
    update_rate = rospy.get_param('planers/update_rate',2)
    dead_end_dist_diff_threshhold = rospy.get_param('planers/dead_end_dist_diff_threshhold',2) #in cells
    maximum_jumps = rospy.get_param('planers/maximum_jumps',500)
    consecutive_jumps_threshhold = rospy.get_param('planers/consecutive_jumps_threshhold',5)
    robot_pos_topic =  rospy.get_param('planers/robot_pos_topic',"/odom")
    debug = rospy.get_param('planers/debug',1)
    dist_to_target_threshhold =  rospy.get_param('planers/global_dist_to_target_threshhold',2) #in cells
    step = rospy.get_param('planers/step',4) #in cells (with resolution 2x2 step of 1 = 2cm)
    step_circle_resolution = rospy.get_param('planers/step_circle_resolution', 3)  #number of slices on circle circumference (more = more round)
    path_publish_topic =  rospy.get_param('planers/path_publish_topic', 'global_path')
    pose_subscribe_topic =  rospy.get_param('planers/pose_subscribe_topic', 'target_pose')
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
    costmap = [] 
    costmap_resolution = 0
    target = np.array([0,0,0])
    num_jumps = 0
    consecutive_jumps = 0
    poses = dCoordsOnCircle(step, step_circle_resolution)
    @staticmethod
    def setNew(new_Global = [0,0,0]): #new_Global is a list [x,y,th] where x and y are cells on costmap
        Global.list.clear()
        Global.goal_reached = 0
        Global.target = np.array(new_Global)
        Global.costmap = [[]] #here we shoudl retrieve global_costmap from server
        #!!!!!!!!!!!!!!
        Global.list.append(Global.robot_pos[:2],0) #Здесь нужно получить по ебалу от негров, потому объявление войны было ошибкой!
        Global.start_pos = Global.robot_pos
        #!!!!!!!!!!!!!!
    @staticmethod
    def reset(): #For reset service
        blank = np.array([0,0,0])
        Global.target = blank
        Global.robot_pos = blank
    @staticmethod
    def appendNextPos(): #uses only x and y, the needed orientations should be set after the goal list is complete
        current_pos = np.array(Global.list.pop(-1)[:2])  #this is the last and current position
        target_vect = np.array(Global.target[:2]) - current_pos
        delta_vect = target_vect/np.linalg.norm(target_vect) * Global.step
        next_pos = current_pos + delta_vect
        Global.num_jumps += 1
        for num,x,y in enumerate(Global.poses):
            current_dist = np.linalg.norm(next_pos - Global.start_pos[:2])
            if Global.num_jumps > Global.maximum_jumps:                                               #check for jumps overflow
                rospy.logerror(f"Jumps > maximum({Global.maximum_jumps})")
                Global.goal_reached = 1
                break
            if Global.costmap[next_pos[0]][next_pos[1]] < Global.maximum_cost:                        #if next cell is acceptable then
                if np.linalg.norm(Global.target - current_pos) < Global.dist_to_target_threshhold:    #its checked to be close enough to the goal
                    Global.num_jumps = 0                                                              #if too close - ignored, last target added
                    Global.goal_reached = 1
                    Global.list.append((Global.target,current_dist))
                    break
                elif Global.consecutive_jumps > Global.consecutive_jumps_threshhold:                  #additional point is added in case of long straight jump
                    Global.list.append((current_pos,current_dist))   
                    Global.consecutive_jumps = 0                                                      #consecutive jumps counter reset
                    break
                elif num > 0:                                                                                          
                    Global.list.append((next_pos,np.linalg.norm(next_pos - Global.start_pos[:2])))    #if a turn is made, the laast point is also saved
                    Global.consecutive_jumps += 1
                    break
                else:    
                    Global.list.append((current_pos,current_dist))                                    #otherwise remove current, add next
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
        for num, vect, dist in enumerate(Global.list):
            if dist > max_dist:
                max_dist = dist
            else:
                for _ ,subdist in Global.list[:num]:
                    if subdist-dist>Global.dead_end_dist_diff_threshhold:
                        list_to_remove.append((vect,dist))
        for val in list_to_remove:
            Global.list.remove(val)
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
        target = Global.list.pop()
        for goal,_ in Global.list:
                pose = PoseStamped()
                pose.pose.position.x = goal[0]
                pose.pose.position.y = goal[1]
                Global.sendTransfrom(goal[0],goal[1],0)
                msg.poses.append(pose) 
        target_pos = PoseStamped()
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, target[2])
        target_pos.pose.orientation.x = quaternion[0]
        target_pos.pose.orientation.y = quaternion[1]
        target_pos.pose.orientation.z = quaternion[2]
        target_pos.pose.orientation.w = quaternion[3]
        target_pos.pose.position.x = target[0]
        target_pos.pose.position.y = target[1]
        Global.sendTransfrom(target[0],target[1],target[2])
        msg.poses.append(target_pos)
        Global.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Global.list)+1} points") 
        