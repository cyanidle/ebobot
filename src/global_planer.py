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
    
switch = 1
def switchGenerator():
    yield switch * -1
class Goal: ##Полная жопа, я не ебу как это реализовать, через ООП или нет, пока что так
    #Params
    step = rospy.get_param('global_planer/step',0.1)
    step_radians = rospy.get_param('global_planer/step_radians', 0.03)
    #Params
    list = []
    costmap = []
    target = np.array([0,0,0])
    def setNew(new_goal): #new_goal is a np.array[x,y,theta]
        Goal.list.clear()
        Goal.target = new_goal
        Goal.costmap = [[],[]] #here we shoudl retrieve global_costmap from server
        #!!!!!!!!!!!!!!
        Goal.list.append(robot_pos) #Здесь нужно получить новые актуальные координаты ебобота!!!!!!!!!!
        #!!!!!!!!!!!!!!

    def getNextPos():
        current_pos = Goal.list.pop(1)  #this is the last and current position
        target_vect = Goal.target - current_pos
        norm = target_vect/np.max(np.abs(target_vect))
        next_pos = norm * Goal.target + current_pos
        delta_vect = next_pos - current_pos
        delta_complex = delta_vect[0] + 1j*delta_vect[1]
        for dir in switchGenerator():
            if Goal.costmap[delta_complex.real+current_pos[0]][delta_complex.imag+current_pos[1]] == 0:
                break
            turn = dir * Goal.step_radians
            delta_complex = (cmath.sin(turn) + cmath.cos(turn)) * delta_complex    
        #convert complex delta_vector back to real
        #add delta vector to current pos
            
        

    

if __name__ == "__main__":
    #pizdec
#Пока что я предполагаю использовать класс: Цель, который хитровыебанным образорм будет добавлять чекпоинты
#В некий внутриклассовый список, чтобы потом его высрать в Path 

#Основной алгоритм - пускаем лучи фиксированной длины в сторону цели, каждый раз добавляя объект "Позиция" в список позиций, и исползуем последнюю позицию, как опору
# !!! Используем Goal.list.pop(1) - это вернет объект вектора последней позиции, запишет в локальную переменную и удалит его из списка
#Когда позциия засчитана, как успешная (не триллион точек по прямой, а только необходимые для огибания препятствия) она записывается в список целей