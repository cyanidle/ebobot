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

#Пусть глобал планер посылает экшоны (goal nav_msgs/Path) в сторону локального и получает некий фидбек по выполнению, в случае ступора он вызвоет либо отдельный скрипт, либо просто некую функцию
#Внутри самого глобал планера, которая временно подтасует текущую цель на "ложную" которая позволит выехать из затруднения (Recovery Behavior)
#В остальное время планеру в тупую следуют указаниям скрипта поведения, посылающего команды в /simple_goal
import Dorvect

# class Dorvect():    
#     def __init__(self,vect = [0,0,0]):
#         self.x = vect[0];self.y = vect[1]; self.th = vect[2]
#         self.vect = [self.x,self.y,self.th]
#         self.imag = self.x + 1j*self.y
#     def updateFromImag(self,imag):
#         self = Dorvect([imag.real,imag.imag,self.th])
#     def dist(self):
#         return pow(self.x**2+self.y**2,0.5)
#     #def toImag(self):
#         #self.imag = self.x + 1j*self.y
#         #return self.imag
#     def norm2d(self):
#         return max([abs(self.x),abs(self.y)])
#     def norm3d(self):
#         return max([abs(self.x),abs(self.y),abs(self.th)])
#     def normalized3d(self):
#         return Dorvect([val/self.norm3d() for val in self.vect])
#     #def max2d(self):
#        # return max([self.x,self.y])
#     #def max3d(self):
#         #return max([self.x,self.y,self.th])
#     def __abs__(self):
#         return Dorvect([abs(self.x),abs(self.y),abs(self.th)])
#     def __str__(self):
#         return f"Dor vector {self.vect}"
#     def __add__(self,other):
#         return Dorvect([self.x + other.x, self.y + other.y,self.th + other.th])
#     def __radd__(self,other):
#         return Dorvect([self.x + other.x, self.y + other.y,self.th + other.th])
#     def __sub__(self, other):
#         return Dorvect([self.x - other.x, self.y - other.y,self.th - other.th])
#     def __mul__(self, other):
#         return Dorvect([self.x * other.x, self.y * other.y,self.th * other.th])
#     def __div__(self, other):
#         return Dorvect([self.x/ other.x, self.y / other.y,self.th/ other.th])






def odomCallback(odom):
    Goal.robot_pos[0] =(odom.pose.position.x)
    Goal.robot_pos[1] =(odom.pose.position.y)
    Goal.robot_pos[2] =(tf.transformations.euler_from_quarternion(odom.pose.orientation)[2])
def targetCallback(target): ######################## ДОДЕЛАТЬ!!!!!!!!
    goal = []
    goal.append(target.pose.position.x)
    goal.append(target.pose.position.y)
    goal.append(tf.transformations.euler_from_quarternion(target.pose.orientation)[2])
    Goal.setNew(goal)

#####################
switch = 1
def switchGenerator():
    switch = switch * -1
    yield switch
#####################


class Goal(): ##Полная жопа, я не ебу как это реализовать, через ООП или нет, пока что так
    #Params
    costmap_resolution = 2 # digits after comma, in meters
    consecutive_jumps_threshhold = rospy.get_param('global_planer/consecutive_jumps_threshhold',5)
    odom_topic =  rospy.get_param('global_planer/odom_topic',"/odom")
    debug = rospy.get_param('global_planer/debug',1)
    threshhold =  rospy.get_param('global_planer/goal_threshhold',0.05)
    step = rospy.get_param('global_planer/step',0.1)
    step_radians = rospy.get_param('global_planer/step_radians', 0.03)
    path_publisher_topic =  rospy.get_param('global_planer/path_publisher_topic', 'global_path')
    path_subscriber_topic =  rospy.get_param('global_planer/path_subscriber_topic', 'target_pose')
    #/Params

    

    robot_pos = [0,0,0]
    list = []
    costmap = []  
    target = Dorvect()
    def setNew(new_goal = [0,0,0]): #new_goal is a list [x,y,th]
        Goal.list.clear()
        Goal.reached = 0
        Goal.target = Dorvect(new_goal)
        Goal.costmap = [[],[]] #here we shoudl retrieve global_costmap from server
        #!!!!!!!!!!!!!!
        Goal.list.append(Dorvect(Goal.robot_pos)) #Здесь нужно получить новые актуальные координаты ебобота!!!!!!!!!!
        #!!!!!!!!!!!!!!

   #@staticmethod
    def appendNextPos():
        current_pos = Goal.list.pop(1)  #this is the last and current position
        target_vect = Goal.target - current_pos
        delta_vect = target_vect.normalized3d() * Goal.step
        next_pos = current_pos + delta_vect
        if (next_pos - Goal.target).dist() < Goal.threshhold:
            Goal.list.append(current_pos)
            Goal.reached = 1
        
        for count, dir in enumerate(switchGenerator()):
            if Goal.costmap[round(next_pos.x,2)][round(next_pos.y,2)] == 0:
                if count > 0 or Goal.consecutive_jumps > Goal.consecutive_jumps_threshhold:
                    Goal.list.append(current_pos)
                    Goal.consecutive_jumps = 0
                else:
                    Goal.list.append(next_pos)
                    Goal.consecutive_jumps += 1
                    break
            turn = dir * Goal.step_radians * count
            if abs(turn) > 4:
                rospy.logwarn(f"Help me stepbro, im stuck")
            next_pos.updateFromImag((cmath.sin(turn) + 1j*cmath.cos(turn)) * next_pos.imag)  
            if Goal.debug:
                rospy.loginfo(f"nextpos = {next_pos}, turn = {turn}")
                if Goal.reached:
                    rospy.logwarn(f"Goal.append called, when goal reached")   
       

    #@staticmethod
    def publish():
        msg = Path()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        for goal in Goal.list:
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
        Goal.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Goal.list)} points") 
        
            
        

    

if __name__ == "__main__":
    #Topics
    odom_subscriber = rospy.Subscriber(Goal.odom_topic, PoseStamped, odomCallback)
    target_subscriber = rospy.Subscriber(Goal.path_subscriber_topic, PoseStamped, targetCallback)
    path_publisher = rospy.Publisher(Goal.path_publisher_topic, Path, queue_size=20)
    #/Topics
    #pizdec
#Пока что я предполагаю использовать класс: Цель, который хитровыебанным образорм будет добавлять чекпоинты
#В некий внутриклассовый список, чтобы потом его высрать в Path 

#Основной алгоритм - пускаем лучи фиксированной длины в сторону цели, каждый раз добавляя объект "ДорВектор" в список позиций, и исползуем последнюю позицию, как опору
# !!! Используем Goal.list.pop(1) - это вернет объект вектора последней позиции, запишет в локальную переменную и удалит его из списка (для очитки при движении по прямой)
#Когда позциия засчитана, как успешная (не триллион точек по прямой, а только необходимые для огибания препятствия) она записывается в список целей
#Если направление прыжка не менялось <порог последовательных прыжков>, то точку все же возвращаем в список

