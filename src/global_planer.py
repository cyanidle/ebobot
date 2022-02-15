#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
#import math
import cmath
import tf
import numpy as np
rospy.init_node('global_planer')
#Messages and actions
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Path, OccupancyGrid, Odometry
######
from libraries.DorLib import Dorvect
######
#Пусть глобал планер посылает экшоны (Global nav_msgs/Path) в сторону локального и получает некий фидбек по выполнению, в случае ступора он вызвоет либо отдельный скрипт, либо просто некую функцию
#Внутри самого глобал планера, которая временно подтасует текущую цель на "ложную" которая позволит выехать из затруднения (Recovery Behavior)
#В остальное время планеру в тупую следуют указаниям скрипта поведения, посылающего команды в /simple_Global


def robotPosCallback(odom):
    Global.robot_pos[0] = Dorvect([odom.pose.position.x,odom.pose.position.y,tf.transformations.euler_from_quarternion(odom.pose.orientation)[2]])

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
    costmap_update_topic = rospy.get_param('global_planer/costmap_topic_update','/costmap_update')
    costmap_topic = rospy.get_param('global_planer/costmap_topic','/costmap')
    maximum_cost = rospy.get_param('global_planer/maximum_cost',30)
    cleanup_feature = rospy.get_param('global_planer/cleanup_feature',1)
    seconds_per_update = rospy.get_param('global_planer/seconds_per_update',0.5)
    dead_end_dist_diff_threshhold = rospy.get_param('global_planer/dead_end_dist_diff_threshhold',0.10)
    maximum_jumps = rospy.get_param('global_planer/maximum_jumps',500)
    consecutive_jumps_threshhold = rospy.get_param('global_planer/consecutive_jumps_threshhold',5)
    robot_pos_topic =  rospy.get_param('global_planer/robot_pos_topic',"/odom")
    debug = rospy.get_param('global_planer/debug',1)
    dist_to_target_threshhold =  rospy.get_param('global_planer/global_dist_to_target_threshhold',0.12)
    step = rospy.get_param('global_planer/step',0.2)
    step_radians = rospy.get_param('global_planer/step_radians', 0.1) 
    path_publish_topic =  rospy.get_param('global_planer/path_publish_topic', 'global_path')
    pose_subscribe_topic =  rospy.get_param('global_planer/pose_subscribe_topic', 'target_pose')
    #/Params
    ################################################ global values
    goal_reached = 1
    start_pos = Dorvect([0,0,0])
    robot_pos = Dorvect([0,0,0])
    list = []
    costmap = [] 
    costmap_resolution = 0
    target = Dorvect()
    num_jumps = 0
    ################################################
    switch = 1
    next_switch_dir = 0
    def switchGenerator():
        Global.switch = Global.switch * -1
        if Global.next_switch_dir:
            yield Global.next_switch_dir
        else:
            yield Global.switch
    ################################################
    def setNew(new_Global = [0,0,0]): #new_Global is a list [x,y,th]
        Global.list.clear()
        Global.goal_reached = 0
        Global.target = Dorvect(new_Global)
        Global.costmap = [[]] #here we shoudl retrieve global_costmap from server
        #!!!!!!!!!!!!!!
        Global.list.append(Global.robot_pos,0) #Здесь нужно получить новые актуальные координаты ебобота!!!!!!!!!!
        Global.start_pos = Global.robot_pos
        #!!!!!!!!!!!!!!
    def reset():
        blank = Dorvect([0,0,0])
        Global.target = blank
        Global.robot_pos = blank
   #@staticmethod
    def appendNextPos(): ###### Самый пиздец
        current_pos = Global.list.pop(-1)  #this is the last and current position
        target_vect = Global.target - current_pos
        delta_vect = target_vect.normalized3d() * Global.step
        next_pos = current_pos + delta_vect
        Global.num_jumps += 1
        for count, dir in enumerate(Global.switchGenerator()):
            if Global.num_jumps > Global.maximum_jumps:
                rospy.logerror(f"Jumps > {Global.maximum_jumps}")
                Global.goal_reached = 1
            if Global.costmap[next_pos.x//Global.costmap_resolution][next_pos.y//Global.costmap_resolution] < Global.maximum_cost:
                if Dorvect.dist(Global.target - current_pos) < Global.dist_to_target_threshhold:
                    Global.num_jumps = 0
                    Global.goal_reached = 1
                    Global.list.append((Global.target,Dorvect.dist(current_pos - Global.start_pos)))
                elif count > 0 or Global.consecutive_jumps > Global.consecutive_jumps_threshhold:
                    Global.list.append((current_pos,Dorvect.dist(current_pos - Global.start_pos)))
                    Global.consecutive_jumps = 0
                else:
                    Global.list.append((next_pos,Dorvect.dist(next_pos - Global.start_pos)))
                    Global.consecutive_jumps += 1
                    break
            turn = dir * Global.step_radians * (count//2)
            if abs(turn) > 4:
                rospy.logerror(f"Help me stepbro, im stuck")
                if Global.debug:
                    Global.reset()
                break
            delta_vect.updateFromImag((cmath.sin(turn) + 1j*cmath.cos(turn)) * next_pos.imag)
            next_pos = current_pos + delta_vect
            if Global.debug:
                rospy.loginfo(f"nextpos = {next_pos}, turn = {turn}")
                if Global.goal_reached:
                    rospy.logwarn(f"Global.append called, when Global reached")   
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
    #@staticmethod
    def publish():
        msg = Path()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        for goal,_ in Global.list:
                pose = PoseStamped()
                pose.pose.position.x = goal.vect[0]
                pose.pose.position.y = goal.vect[1]
                pose.pose.position.z = 0
                quaternion = tf.transformations.quaternion_from_euler(0, 0, goal.vect[2])
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                msg.poses.append(pose) 
        Global.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Global.list)} points") 
        
            
        

    

if __name__ == "__main__":
    #Topics
    costmap_update_subscriber = rospy.Subscriber(Global.costmap_update_topic, OccupancyGridUpdate, costmapUpdateCallback)
    costmap_subscriber = rospy.Subscriber(Global.costmap_topic, OccupancyGrid, costmapCallback)
    robot_pos_subscriber = rospy.Subscriber(Global.robot_pos_topic, PoseStamped, robotPosCallback)
    target_subscriber = rospy.Subscriber(Global.pose_subscribe_topic, PoseStamped, targetCallback)
    path_publisher = rospy.Publisher(Global.path_publish_topic, Path, queue_size=10)
    #/Topics
    rospy.sleep(1)
    while not not rospy.is_shutdown():
        if not Global.goal_reached:
            while not Global.goal_reached:
                Global.appendNextPos()
            if Global.cleanup_feature:
                Global.cleanupDeadEnds()
            Global.publish()
        rospy.sleep(Global.seconds_per_update)



    
    #pizdec
#Пока что я предполагаю использовать класс: Global, который хитровыебанным образорм будет добавлять чекпоинты
#В некий внутриклассовый список, чтобы потом его высрать в Path 

#Основной алгоритм - пускаем лучи фиксированной длины в сторону цели, каждый раз добавляя объект "ДорВектор" в список позиций tuple(vect, dist), и исползуем последнюю позицию, как опору
# !!! Используем Global.list.pop(-1) - это вернет объект вектора последней позиции, запишет в локальную переменную и удалит его из списка (для очитки при движении по прямой)
#Когда позциия засчитана, как успешная (не триллион точек по прямой, а только необходимые для огибания препятствия) она записывается в список целей
#Если направление прыжка не менялось <порог последовательных прыжков>, то точку все же возвращаем в список

