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
    footprint_radius = 50
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
    d_time = 50

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
            Motors.delta_x += mot.ddist * math.cos(Motors.theta + mot.radians)
            Motors.delta_y += mot.ddist * math.sin(Motors.theta + mot.radians)
        Motors.last_x = Motors.x
        Motors.x += Motors.delta_x
        Motors.last_y = Motors.y
        Motors.y += Motors.delta_y
        Motors.vx = (Motors.x - Motors.last_x) * 1000/Motors.d_time
        Motors.vy = (Motors.y - Motors.last_y) * 1000/Motors.d_time




#init motors with their angles
motor1 = Motors(90) 
rospy.loginfo(f"Motor 1 initialised with angle - {motor1.angle}, radians - {motor1.radians}, dist - {motor1.dist}"
motor2 = Motors(210)
rospy.loginfo(f"Motor 2 initialised with angle - {motor2.angle}, radians - {motor2.radians}, dist - {motor2.dist}"
motor3 = Motors(330)
rospy.loginfo(f"Motor 3 initialised with angle - {motor3.angle}, radians - {motor3.radians}, dist - {motor3.dist}"
rospy.loginfo(f"Motors list {Motors.list[0]}, {Motors.list[1]}, {Motors.list[2]}"

rate = rospy.Rate(int(1000/Motors.d_time))



def callback(info):
    for mot in Motors.list:
        setattr(mot , curr , info.data[i*4])
        setattr(mot , targ , info.data[i*4 + 1])
        setattr(mot , dist , info.data[i*4 + 2])
        setattr(mot , ddist , info.data[i*4 + 3])

def listener():

    rospy.Subscriber("motors_info", Float32MultiArray, callback)





listener() 
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()


while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    rospy.loginfo(f"Current time = {current_time}")
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, Motors.theta)
    odom_broadcaster.sendTransform(
        (Motors.x, Motors.y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
        )
    rospy.spin()
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(Motors.x, Motors.y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(Motors.vx, Motors.vy, 0), Vector3(0, 0, Motors.theta))
    odom_pub.publish(odom)
    last_time = current_time
    rate.sleep()
