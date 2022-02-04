#!/usr/bin/env python3
import roslib; 
import rospy
from std_msgs.msg import Float32MultiArray
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
rospy.init_node('motors_info_callback', anonymous=True)


debug = 0 #довольно приятно иногда офнуть нечитаемый мусор, можно потом в параметр запуска перевестиЫ
info_len = 12
current_time = rospy.Time.now()
last_time = rospy.Time.now()


class Motors:
    footprint_rad = 0.15 #in meters
    num = 3
    theta = 0
    x = 0
    y = 0
    list = []
    delta_x = 0
    delta_y = 0
    last_x = 0
    last_y = 0
    spd_x = 0
    spd_y = 0
    Hz = 20 #default updates/second
    vturn = 0
    delta_secs = 0
    duration = 50000 #in nanoseconds (default)
    last_time = rospy.Time.now()
    last_theta = 0

    def __init__(self, num, angle,curr = 0, targ = 0,pwm = 0,ddist = 0):
        self.num = num
        self.curr = curr
        self.targ = targ
        self.pwm = pwm
        self.ddist = ddist
        self.angle = angle
        self.radians = math.radians(angle)
        Motors.list.append(self)
    def updateOdom():               ########### 0.3 m  =  1.8 in odom FIXX
        Motors.duration = rospy.Time.now() - Motors.last_time
        Motors.delta_secs = Motors.duration.to_sec()
        ddist_sum = 0 #reset and calculate new change to theta
        for mot in Motors.list:
            ddist_sum += mot.ddist
        Motors.theta += ddist_sum / Motors.num / Motors.footprint_rad
        Motors.delta_x, Motors.delta_y = 0, 0  #reset and calculate new changes to coords
        for mot in Motors.list:
            Motors.delta_x += mot.ddist * math.cos(Motors.theta + mot.radians) 
            Motors.delta_y += mot.ddist * math.sin(Motors.theta + mot.radians) 
        Motors.last_x = Motors.x
        Motors.x += Motors.delta_x
        Motors.last_y = Motors.y
        Motors.y += Motors.delta_y
        Motors.spd_x, Motors.spd_y =  Motors.delta_x * Motors.delta_secs * Motors.Hz, Motors.delta_y * Motors.delta_secs * Motors.Hz #multiplies change in coords by change in time                                                                         #and number of updates/s
        Motors.vturn = Motors.theta - Motors.last_theta
        Motors.last_theta = Motors.theta 
      
        
        
    



#init motors with their angles
motor0 = Motors(0,90) 
rospy.loginfo(f"Motor 1 initialised with angle - {motor0.angle}, radians - {motor0.radians}")
motor1 = Motors(1,210)
rospy.loginfo(f"Motor 2 initialised with angle - {motor1.angle}, radians - {motor1.radians}")
motor2 = Motors(2,330)
rospy.loginfo(f"Motor 3 initialised with angle - {motor2.angle}, radians - {motor2.radians}")
rospy.loginfo(f"Motors list {[mot.num for mot in Motors.list]}")





def callback(info):
    for mot in Motors.list:
        setattr(mot , "targ" , info.data[getattr(mot,"num")*4])
        setattr(mot , "curr" , info.data[getattr(mot,"num")*4 + 1])
        setattr(mot , "pwm" , info.data[getattr(mot,"num")*4 + 2])
        setattr(mot , "ddist" , info.data[getattr(mot,"num")*4 + 3])
    Motors.updateOdom()
    Motors.last_time = rospy.Time.now()
#######################################################
rospy.Subscriber("motors_info", Float32MultiArray, callback)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

report_count = 0
report_rate = 5 # Hz

rate = rospy.Rate(Motors.Hz) 
while not rospy.is_shutdown():
    report_count += 1
    if debug:
        if report_count > Motors.Hz/report_rate:
            rospy.loginfo(f"Theta = {Motors.theta}, Motors.x = {Motors.x}, Motors.y = {Motors.y}")
            for mot in Motors.list:
                rospy.loginfo(f"Motor {mot.num} current = {mot.curr}, target = {mot.targ}")
            report_count = 0
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
    odom.pose.pose = Pose(Point(Motors.x, Motors.y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(Motors.spd_x, Motors.spd_y, 0), Vector3(0, 0, Motors.vturn))
    odom_pub.publish(odom)
    last_time = current_time
    rate.sleep()
