#!/usr/bin/env python3
import roslib; 
import rospy
from std_msgs.msg import Float32MultiArray
import math
info_len = 12
d_time = 50




class Motors:
    footprint_radius = 50
    num = 3
    theta = 0
    x = 0
    y = 0
    list = []

    def __init__(self, angle,curr = 0, targ = 0,dist = 0,ddist = 0):
        self.curr = curr
        self.targ = targ
        self.dist = dist
        self.ddist = ddist
        self.angle = angle
        self.radians = math.radians(angle)
        Motors.list.append(self)
    def updateOdom():
        ddist_sum = 0
        for mot in Motors.list:
            ddist_sum += mot.ddist
        Motors.theta += ddist_sum/(Motors.num * Motors.footprint_radius)
        for mot in Motors.list:
            Motors.x += mot.ddist * math.cos(Motors.theta + mot.radians)
            Motors.y += mot.ddist * math.sin(Motors.theta + mot.radians)


#init motors with their angles
motor1 = Motors(90) 
motor2 = Motors(210)
motor3 = Motors(330)



def callback(info):
    for i in range(Motors.num):
        Motors.list[i].curr = info.data[i*4]
        Motors.list[i].targ = info.data[i*4 + 1]
        Motors.list[i].dist = info.data[i*4 + 2]
        Motors.list[i].ddist = info.data[i*4 + 3]

def listener():
    #rospy.init_node('motors_info_callback', anonymous=True)
    rospy.Subscriber("motors_info", Float32MultiArray, callback)




if __name__ == '__main__':
    listener() 
    rospy.spin()
