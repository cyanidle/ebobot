#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import numpy as np
import rospy
import tf
from dorlib import dCoordsInRad
######################
from nav_masgs.msg import Odometry
from sensor_msgs.msg import LaserScan
######################
rospy.init_node('laser_scan_callback')
def robotPosCallback(odom):
    quat = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    Laser.robot_pos = np.array([odom.pose.pose.position.y/Laser.costmap_resolution,odom.pose.pose.position.x/Laser.costmap_resolution,tf.transformations.euler_from_quaternion(quat)[2]]) 

def laserScanCallback(scan):
    Laser.angle_min = scan.angle_min
    Laser.angle_max = scan.angle_max
    Laser.angle_increment = scan.angle_increment
    Laser.time_increment = scan.time_increment
    Laser.scan_time  = scan.scan_time 
    Laser.range_min  = scan.range_min
    Laser.range_max = scan.range_max
    Laser.ranges = scan.ranges 
    Laser.intensities  = scan.intensities 
    Laser.update()
class Laser:
    #Params
    #Features
    debug = rospy.get_param("~/debug",1)
    #/Features
    costmap_resolution = rospy.get_param("~/costmap_resolution",0.02)
    #/Params

    #Topics
    laser_scan_topic = rospy.get_param("~/laser_scan_topic", "/laser_scan")
    robot_pos_topic = rospy.get_param("~/robot_pos_topic", "/odom")
    #
    laser_scan_subscriber = rospy.Subsrciber(laser_scan_topic,LaserScan,laserScanCallback)
    robot_pos_subscriber = rospy.Subsrciber(robot_pos_topic,Odometry,robotPosCallback)
    #/Topics


    #Global values
    angle_min = 0
    angle_max = 0
    angle_increment = 0
    time_increment = 0
    scan_time = 0
    range_min = 0
    range_max = 0
    ranges = []
    intensities = []
    costmap = []
    robot_pos = np.array([0,0,0])
    #/Global values
    
    @classmethod
    def update(cls):
        for num, pair in enumerate(zip(cls.ranges, cls.intensities)):
            range, intencity = pair