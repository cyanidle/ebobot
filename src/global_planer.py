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
from nav_msgs.msg import Path, OccupancyGrid
######

#Пусть глобал планер посылает экшоны (goal nav_msgs/Path) в сторону локального и получает некий фидбек по выполнению, в случае ступора он вызвоет либо отдельный скрипт, либо просто некую функцию
#Внутри самого глобал планера, которая временно подтасует текущую цель на "ложную" которая позволит выехать из затруднения (Recovery Behavior)
#В остальное время планеру в тупую следуют указаниям скрипта поведения, посылающего команды в /simple_goal

class Dorvect():    
    def __init__(self,vect = [0,0,0]):
        self.x = vect[0];self.y = vect[1]; self.th = vect[2]
        self.vect = [self.x,self.y,self.th]
        self.imag = self.x + 1j*self.y
    def updateFromImag(self,imag):
        self = Dorvect([imag.real,imag.imag,self.th])
    #def toImag(self):
        #self.imag = self.x + 1j*self.y
        #return self.imag
    def norm2d(self):
        return max([abs(self.x),abs(self.y)])
    def norm3d(self):
        return max([abs(self.x),abs(self.y),abs(self.th)])
    def normalized3d(self):
        return Dorvect([val/self.norm3d() for val in self.vect])
    #def max2d(self):
       # return max([self.x,self.y])
    #def max3d(self):
        #return max([self.x,self.y,self.th])
    def __abs__(self):
        return Dorvect([abs(self.x),abs(self.y),abs(self.th)])
    def __str__(self):
        return f"Dor vector {self.vect}"
    def __add__(self,other):
        return Dorvect([self.x + other.x, self.y + other.y,self.th + other.th])
    def __radd__(self,other):
        return Dorvect([self.x + other.x, self.y + other.y,self.th + other.th])
    def __sub__(self, other):
        return Dorvect([self.x - other.x, self.y - other.y,self.th - other.th])
    def __mul__(self, other):
        return Dorvect([self.x * other.x, self.y * other.y,self.th * other.th])
    def __div__(self, other):
        return Dorvect([self.x/ other.x, self.y / other.y,self.th/ other.th])
    
def poseCallback(): ######################## ДОДЕЛАТЬ!!!!!!!!!
    return

switch = 1
def switchGenerator():
    yield switch * -1
class Goal(): ##Полная жопа, я не ебу как это реализовать, через ООП или нет, пока что так
    #Params
    step = rospy.get_param('global_planer/step',0.1)
    step_radians = rospy.get_param('global_planer/step_radians', 0.03)
    path_publisher_topic =  rospy.get_param('global_planer/path_publisher_topic', 'global_path')
    path_subscriber_topic =  rospy.get_param('global_planer/path_subscriber_topic', 'target_pose')
    #/Params
    target_subscriber = rospy.Subscriber(path_subscriber_topic, PoseStamped, poseCallback)
    path_publisher = rospy.Publisher(path_publisher_topic, Path, queue_size=20)
    list = []
    costmap = []  
    target = Dorvect()
    def setNew(new_goal = [0,0,0]): #new_goal is a list [x,y,th]
        Goal.list.clear()
        Goal.target = Dorvect(new_goal)
        Goal.costmap = [[],[]] #here we shoudl retrieve global_costmap from server
        #!!!!!!!!!!!!!!
        Goal.list.append(Dorvect(robot_pos)) #Здесь нужно получить новые актуальные координаты ебобота!!!!!!!!!!
        #!!!!!!!!!!!!!!

    def getNextPos():
        current_pos = Goal.list.pop(1)  #this is the last and current position
        target_vect = Goal.target - current_pos
        norm = target_vect/np.max(np.abs(target_vect))
        delta_vect = target_vect.normalized3d() * Goal.step
        next_pos = current_pos + delta_vect
        for count, dir in enumerate(switchGenerator()):
            if Goal.costmap[round(next_pos.x,2)][round(next_pos.y,2)] == 0:
                if count > 0:
                    Goal.list.append(current_pos)
                Goal.list.append(next_pos)
                break
            turn = dir * Goal.step_radians * count
            next_pos.updateFromImag((cmath.sin(turn) + 1j*cmath.cos(turn)) * next_pos.imag)

        #### ADD a check whether the goal is reached

 
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
    #pizdec
#Пока что я предполагаю использовать класс: Цель, который хитровыебанным образорм будет добавлять чекпоинты
#В некий внутриклассовый список, чтобы потом его высрать в Path 

#Основной алгоритм - пускаем лучи фиксированной длины в сторону цели, каждый раз добавляя объект "Позиция" в список позиций, и исползуем последнюю позицию, как опору
# !!! Используем Goal.list.pop(1) - это вернет объект вектора последней позиции, запишет в локальную переменную и удалит его из списка
#Когда позциия засчитана, как успешная (не триллион точек по прямой, а только необходимые для огибания препятствия) она записывается в список целей
