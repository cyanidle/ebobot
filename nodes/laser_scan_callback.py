#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import numpy as np
from math import cos,sin
import rospy
import tf
from dorlib import dCoordsInRad
######################
from nav_masgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
######################
rospy.init_node('laser_scan_callback')
def robotPosCallback(odom):
    quat = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    Laser.robot_pos = np.array([odom.pose.pose.position.y/Laser.costmap_resolution,odom.pose.pose.position.x/Laser.costmap_resolution,tf.transformations.euler_from_quaternion(quat)[2]]) 
def costmapCallback(costmap):
    Laser.costmap_resolution = costmap.info.resolution
    Laser.costmap_height = costmap.info.height
    Laser.costmap_width= costmap.info.width
    rospy.loginfo_once(f"Got new map, height = {Laser.costmap_height}, width= {Laser.costmap_width}")
    Laser.costmap= np.reshape(costmap.data,(Laser.costmap_height, Laser.costmap_width))  
    #Laser.debug_map = Laser.costmap
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
    if not Laser.angles_done:
        Laser.precalcCoeffs()
    Laser.update()
class Laser:
    #Params
    #Features
    debug = rospy.get_param("~/debug",1)
    #/Features
    costmap_resolution = rospy.get_param("~/costmap_resolution",0.02)
    #/Params

    #Topics
    costmap_topic = rospy.get_param('~/costmap_topic','/costmap')
    laser_scan_topic = rospy.get_param("~/laser_scan_topic", "/laser_scan")
    robot_pos_topic = rospy.get_param("~/robot_pos_topic", "/odom")
    #
    costmap_subscriber = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback)
    laser_scan_subscriber = rospy.Subsrciber(laser_scan_topic,LaserScan,laserScanCallback)
    robot_pos_subscriber = rospy.Subsrciber(robot_pos_topic,Odometry,robotPosCallback)
    #/Topics


    #Global values
    angles_done = 0
    coeffs =[]
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
    def precalcCoeffs(cls):
        for num in range(Laser.angle_min, Laser.angle_max):
            Laser.coeffs.append((sin(Laser.angle_increment*num),cos(Laser.angle_increment*num))) 
        cls.angles_done = 1
    #####################
    @classmethod
    def update(cls):
        for range, intensity, coeffs in zip(cls.ranges, cls.intensities, cls.coeffs):
            y_coeff, x_coeff = coeffs
            meters_pos = np.array((range * y_coeff, range * x_coeff))
            pixel_pos = meters_pos/cls.costmap_resolution


class Beacons:
    #Beacon params
    dist_threshhold = rospy.get_param('costmap_server/beacons/dist_threshhold',2) #in cells
    dist_between_double = rospy.get_param('costmap_server/beacons/dist_between_double',80) #in cells
    dist_from_double = rospy.get_param('costmap_server/beacons/dist_from_double', 155) #in cells (median of base of the triangle)
    #/Beacon params
    new_coords = []
    list = []
    pose = (0,0)
    def __init__(self,pos:tuple):
        self.pose = pos
        Beacons.list.append(self)
    @classmethod
    def recieve(cls):
        Beacons.list.clear()
        for pose in cls.new_coords:
            beacon = Beacons(pose)