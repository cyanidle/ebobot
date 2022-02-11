import roslib
roslib.load_manifest('ebobot')
import rospy
#import math
import cmath
import tf
import numpy as np
rospy.init_node('global_planer')
#Messages and actions
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Path, OccupancyGrid, Odometry
######

#Пусть глобал планер посылает экшоны (Global nav_msgs/Path) в сторону локального и получает некий фидбек по выполнению, в случае ступора он вызвоет либо отдельный скрипт, либо просто некую функцию
#Внутри самого глобал планера, которая временно подтасует текущую цель на "ложную" которая позволит выехать из затруднения (Recovery Behavior)
#В остальное время планеру в тупую следуют указаниям скрипта поведения, посылающего команды в /simple_Global
from libraries import Dorvect

def odomCallback(odom):
    Global.robot_pos[0] =(odom.pose.position.x)
    Global.robot_pos[1] =(odom.pose.position.y)
    Global.robot_pos[2] =(tf.transformations.euler_from_quarternion(odom.pose.orientation)[2])
def targetCallback(target): ######################## ДОДЕЛАТЬ!!!!!!!!
    Global = []
    Global.append(target.pose.position.x)
    Global.append(target.pose.position.y)
    Global.append(tf.transformations.euler_from_quarternion(target.pose.orientation)[2])
    Global.setNew(Global)

#####################
switch = 1
def switchGenerator():
    switch = switch * -1
    yield switch
#####################


class Global(): ##Полная жопа, я не ебу как это реализовать, через ООП или нет, пока что так
    #Params
    costmap_resolution = 2 # digits after comma, in meters
    consecutive_jumps_threshhold = rospy.get_param('global_planer/consecutive_jumps_threshhold',5)
    odom_topic =  rospy.get_param('global_planer/odom_topic',"/odom")
    debug = rospy.get_param('global_planer/debug',1)
    threshhold =  rospy.get_param('global_planer/Global_threshhold',0.05)
    step = rospy.get_param('global_planer/step',0.1)
    step_radians = rospy.get_param('global_planer/step_radians', 0.03)
    path_publisher_topic =  rospy.get_param('global_planer/path_publisher_topic', 'global_path')
    pose_subscriber_topic =  rospy.get_param('global_planer/pose_subscriber_topic', 'target_pose')
    #/Params

    

    robot_pos = [0,0,0]
    list = []
    costmap = []  
    target = Dorvect()
    def setNew(new_Global = [0,0,0]): #new_Global is a list [x,y,th]
        Global.list.clear()
        Global.reached = 0
        Global.target = Dorvect(new_Global)
        Global.costmap = [[],[]] #here we shoudl retrieve global_costmap from server
        #!!!!!!!!!!!!!!
        Global.list.append(Dorvect(Global.robot_pos)) #Здесь нужно получить новые актуальные координаты ебобота!!!!!!!!!!
        #!!!!!!!!!!!!!!

   #@staticmethod
    def appendNextPos():
        current_pos = Global.list.pop(1)  #this is the last and current position
        target_vect = Global.target - current_pos
        delta_vect = target_vect.normalized3d() * Global.step
        next_pos = current_pos + delta_vect
        if (next_pos - Global.target).dist() < Global.threshhold:
            Global.list.append(current_pos)
            Global.reached = 1
        
        for count, dir in enumerate(switchGenerator()):
            if Global.costmap[round(next_pos.x,2)][round(next_pos.y,2)] == 0:
                if count > 0 or Global.consecutive_jumps > Global.consecutive_jumps_threshhold:
                    Global.list.append(current_pos)
                    Global.consecutive_jumps = 0
                else:
                    Global.list.append(next_pos)
                    Global.consecutive_jumps += 1
                    break
            turn = dir * Global.step_radians * count
            if abs(turn) > 4:
                rospy.logwarn(f"Help me stepbro, im stuck")
            next_pos.updateFromImag((cmath.sin(turn) + 1j*cmath.cos(turn)) * next_pos.imag)  
            if Global.debug:
                rospy.loginfo(f"nextpos = {next_pos}, turn = {turn}")
                if Global.reached:
                    rospy.logwarn(f"Global.append called, when Global reached")   
       

    #@staticmethod
    def publish():
        msg = Path()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        for goal in Global.list:
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
        goal.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Global.list)} points") 
        
            
        

    

if __name__ == "__main__":
    #Topics
    odom_subscriber = rospy.Subscriber(Global.odom_topic, PoseStamped, odomCallback)
    target_subscriber = rospy.Subscriber(Global.pose_subscriber_topic, PoseStamped, targetCallback)
    path_publisher = rospy.Publisher(Global.path_publisher_topic, Path, queue_size=20)
    #/Topics



    
    #pizdec
#Пока что я предполагаю использовать класс: Цель, который хитровыебанным образорм будет добавлять чекпоинты
#В некий внутриклассовый список, чтобы потом его высрать в Path 

#Основной алгоритм - пускаем лучи фиксированной длины в сторону цели, каждый раз добавляя объект "ДорВектор" в список позиций, и исползуем последнюю позицию, как опору
# !!! Используем Global.list.pop(1) - это вернет объект вектора последней позиции, запишет в локальную переменную и удалит его из списка (для очитки при движении по прямой)
#Когда позциия засчитана, как успешная (не триллион точек по прямой, а только необходимые для огибания препятствия) она записывается в список целей
#Если направление прыжка не менялось <порог последовательных прыжков>, то точку все же возвращаем в список

