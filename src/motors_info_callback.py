#!/usr/bin/env python3
import roslib; 
import rospy
from std_msgs.msg import Float32MultiArray
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import sin, cos, pi
from nav_msgs.msg import Odometry
rospy.init_node('motors_info_callback', anonymous=True)

info_len = 12
current_time = rospy.Time.now()
last_time = rospy.Time.now()


class Motors:
    footprint_len = 0.55 #in meters
    num = 3
    theta = 0
    x = 0
    y = 0
    list = []
    delta_x = 0
    delta_y = 0
    last_x = 0
    last_y = 0
    vx = 0
    vy = 0
    d_time = 50 #in milliseconds

    def __init__(self, num, angle,curr = 0, targ = 0,pwm = 0,ddist = 0):
        self.num = num
        self.curr = curr
        self.targ = targ
        self.pwm = pwm
        self.ddist = ddist
        self.angle = angle
        self.radians = math.radians(angle)
        Motors.list.append(self)
    def updateOdom():
        ddist_sum = 0
        for mot in Motors.list:
            ddist_sum += mot.ddist
        Motors.theta += ddist_sum/(100*Motors.num * Motors.footprint_len)
        for mot in Motors.list:
            Motors.delta_x += mot.ddist * math.cos(Motors.theta + mot.radians)
            Motors.delta_y += mot.ddist * math.sin(Motors.theta + mot.radians)
        Motors.last_x = Motors.x
        Motors.x += Motors.delta_x
        Motors.last_y = Motors.y
        Motors.y += Motors.delta_y
        Motors.vx = (Motors.x - Motors.last_x) * 1000/Motors.d_time
        Motors.vy = (Motors.y - Motors.last_y) * 1000/Motors.d_time
        #Counted from odom, not curr speed of motors



#init motors with their angles
motor0 = Motors(0,90) 
rospy.loginfo(f"Motor 1 initialised with angle - {motor0.angle}, radians - {motor0.radians}, pwm - {motor0.pwm}")
motor1 = Motors(1,210)
rospy.loginfo(f"Motor 2 initialised with angle - {motor1.angle}, radians - {motor1.radians}, pwm - {motor1.pwm}")
motor2 = Motors(2,330)
rospy.loginfo(f"Motor 3 initialised with angle - {motor2.angle}, radians - {motor2.radians}, pwm - {motor2.pwm}")
rospy.loginfo(f"Motors list {Motors.list[0].num}, {Motors.list[1].num}, {Motors.list[2].num}")

rate = rospy.Rate(int(1000/Motors.d_time))



def callback(info):
    for mot in Motors.list:
        setattr(mot , "targ" , info.data[getattr(mot,"num")*4])
        setattr(mot , "curr" , info.data[getattr(mot,"num")*4 + 1])
        setattr(mot , "pwm" , info.data[getattr(mot,"num")*4 + 2])
        setattr(mot , "ddist" , info.data[getattr(mot,"num")*4 + 3])
    Motors.updateOdom()

def listener():

    rospy.Subscriber("motors_info", Float32MultiArray, callback)





listener() 
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()


while not rospy.is_shutdown():
    
    current_time = rospy.Time.now()
    
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, Motors.theta)
    odom_broadcaster.sendTransform(
        (Motors.x, Motors.y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
        )
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(Motors.vx, Motors.vy, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(Motors.x, Motors.y, 0), Vector3(0, 0, Motors.theta))
    odom_pub.publish(odom)
    last_time = current_time
    rate.sleep()
