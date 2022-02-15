#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
from std_msgs.msg import Float32MultiArray
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
rospy.init_node('motors_info_callback', anonymous=True)


class Motors():
    #Params
    debug = rospy.get_param('motors_info_callback/debug',1) #довольно неприятно, ДА ГДЕ СУКА ОШИБКА
    info_len = rospy.get_param('motors_info_callback/motors_info_len',12) 
    theta_coeff =rospy.get_param('motors_info_callback/theta_coeff',1)
    y_coeff = rospy.get_param('motors_info_callback/y_coeff',1)
    x_coeff = rospy.get_param('motors_info_callback/x_coeff',1)
    wheels_footprint_rad = rospy.get_param('motors_info_callback/wheels_footprint_radius',0.15) #in meters
    #/Params
    num = 3
    theta = 0
    x = 0
    y = 0
    list = []
    last_x = 0
    last_y = 0
    spd_x = 0
    spd_y = 0
    Hz = 20 #default updates/second
    vturn = 0
    spd_turn = 0
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    last_theta = 0



    

    def __init__(self, num, angle,curr = 0, targ = 0,dist = 0,ddist = 0):
        self.num = num
        self.curr = curr
        self.targ = targ
        self.dist = dist
        self.ddist = ddist
        self.angle = angle
        self.radians = math.radians(angle)
        Motors.list.append(self)
    def updateOdom():              
        duration = rospy.Time.now() - Motors.last_time
        delta_secs = duration.to_sec()
        for mot in Motors.list:
            Motors.theta += mot.ddist / Motors.num / (Motors.wheels_footprint_rad) /4 * Motors.theta_coeff  #по че му? (temporary)
        for mot in Motors.list:
            if mot.ddist != 0:
                Motors.x += mot.ddist * math.cos(Motors.theta + mot.radians) / Motors.num /2 * Motors.x_coeff#temporary
                Motors.y += mot.ddist * math.sin(Motors.theta + mot.radians) / Motors.num /2 * Motors.y_coeff#temporary
        Motors.last_x, Motors.last_y, Motors.last_theta = Motors.x, Motors.y, Motors.theta
        Motors.spd_x, Motors.spd_y =  (Motors.x - Motors.last_x) * delta_secs * Motors.Hz, (Motors.y - Motors.last_y) * delta_secs * Motors.Hz #multiplies change in coords by change in time                                                                         #and number of updates/s
        Motors.spd_turn = Motors.theta - Motors.last_theta
        Motors.theta = Motors.theta % (2 * math.pi)
      




def callback(info):
    for mot in Motors.list:
        setattr(mot , "targ" , info.data[getattr(mot,"num")*4])
        setattr(mot , "curr" , info.data[getattr(mot,"num")*4 + 1])
        setattr(mot , "dist" , info.data[getattr(mot,"num")*4 + 2])
        setattr(mot , "ddist" , info.data[getattr(mot,"num")*4 + 3])
    Motors.updateOdom()
    Motors.last_time = rospy.Time.now()
#######################################################
if __name__=="__main__":
    motors_info_subscriber = rospy.Subscriber("motors_info", Float32MultiArray, callback)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(Motors.Hz)
    report_count = 0

    #init motors with their angles
    motor0 = Motors(0,90) 
    rospy.loginfo(f"Motor 1 initialised with angle - {motor0.angle}, radians - {motor0.radians}")
    motor1 = Motors(1,210)
    rospy.loginfo(f"Motor 2 initialised with angle - {motor1.angle}, radians - {motor1.radians}")
    motor2 = Motors(2,330)
    rospy.loginfo(f"Motor 3 initialised with angle - {motor2.angle}, radians - {motor2.radians}")
    rospy.loginfo(f"Motors list {[mot.num for mot in Motors.list]}")
    rospy.loginfo(f"Dorlib: {dir(Dorlib)}")
    ###################
    rospy.sleep(1)
    while not rospy.is_shutdown():
        
        if Motors.debug:
            report_count += 1
            if report_count > 5:
                report_count = 0
                rospy.loginfo(f"motor 1: {motor0.curr}, {motor0.targ},{motor0.dist},{motor0.ddist}")
        
        
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
        odom.twist.twist = Twist(Vector3(Motors.spd_x, Motors.spd_y, 0), Vector3(0, 0, Motors.spd_turn))
        odom_pub.publish(odom)
        last_time = current_time
        rate.sleep()
