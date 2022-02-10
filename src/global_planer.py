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
from nav_msgs.msg import Path
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
        return max(self.__abs__(self))
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
    


switch = 1
def switchGenerator():
    yield switch * -1
class Goal(): ##Полная жопа, я не ебу как это реализовать, через ООП или нет, пока что так
    #Params
    step = rospy.get_param('global_planer/step',0.1)
    step_radians = rospy.get_param('global_planer/step_radians', 0.03)
    #Params
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
        count = 0
        for dir in switchGenerator():
            count += 1
            if Goal.costmap[next_pos.x][next_pos.y] == 0:
                if count > 1:
                    Goal.list.append(current_pos)
                Goal.list.append(next_pos)
                break
            turn = dir * Goal.step_radians * count
            next_pos.updateFromImag((cmath.sin(turn) + 1j*cmath.cos(turn)) * next_pos.imag)    
        
        
            
        

    

if __name__ == "__main__":
    #pizdec
#Пока что я предполагаю использовать класс: Цель, который хитровыебанным образорм будет добавлять чекпоинты
#В некий внутриклассовый список, чтобы потом его высрать в Path 

#Основной алгоритм - пускаем лучи фиксированной длины в сторону цели, каждый раз добавляя объект "Позиция" в список позиций, и исползуем последнюю позицию, как опору
# !!! Используем Goal.list.pop(1) - это вернет объект вектора последней позиции, запишет в локальную переменную и удалит его из списка
#Когда позциия засчитана, как успешная (не триллион точек по прямой, а только необходимые для огибания препятствия) она записывается в список целей
