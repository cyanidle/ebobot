#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import numpy as np
from math import cos,sin
import rospy
import tf
#####################
from dorlib import dCoordsInRad, turnVect
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
    #Features
    debug = rospy.get_param("~/debug",1)
    #/Features

    #Params
    update_rate = rospy.get_param("~/update_rate",5) #updates/sec
    rads_offset = rospy.get_param("~/rads_offset",1.5) #in radians diff from lidar`s 0 rads and costmap`s in default position(depends where lidars each scan starts, counterclockwise)
    costmap_resolution = rospy.get_param("~/costmap_resolution",0.02) #meters/cell
    #/Params

    #Topics
    costmap_topic = rospy.get_param('~/costmap_topic','/costmap')
    laser_scan_topic = rospy.get_param("~/laser_scan_topic", "/laser_scan")
    robot_pos_topic = rospy.get_param("~/robot_pos_topic", "/odom")
    #
    laser_broadcaster = tf.TransformBroadcaster()
    costmap_subscriber = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback)
    laser_scan_subscriber = rospy.Subsrciber(laser_scan_topic,LaserScan,laserScanCallback)
    robot_pos_subscriber = rospy.Subsrciber(robot_pos_topic,Odometry,robotPosCallback)
    #/Topics


    #Global values
    abs_list = []
    prob_list = []
    #
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
            Laser.coeffs.append((cos(Laser.angle_increment*num),sin(Laser.angle_increment*num))) 
        cls.angles_done = 1
    #####################
    @classmethod
    def updateTF(cls):
        quat = tf.transformations.quaternion_from_euler(0,0,-cls.robot_pos[2])
        cls.laser_broadcaster.sendTransform(
            (cls.robot_pos[1]*cls.costmap_resolution, cls.robot_pos[0]*cls.costmap_resolution, 0),
            quat,
            rospy.Time.now(),
            "laser",
            "costmap" ##???
        )
    ###############
    @classmethod
    def update(cls):
        cls.abs_list.clear()
        cls.prob_list.clear()
        cls.updateTF()
        for range, intensity, coeffs in zip(cls.ranges, cls.intensities, cls.coeffs):
            y_coeff, x_coeff = coeffs
            if range < cls.range_max and range > cls.range_min:
                meters_pos = np.array((range * y_coeff, range * x_coeff))
                abs_pixel_pos = meters_pos/cls.costmap_resolution
                prob_pixel_pos = np.array(turnVect(abs_pixel_pos - cls.robot_pos[:2],  -cls.robot_pos[2]))
                cls.abs_list.append((abs_pixel_pos,intensity))
                cls.prob_list.append((prob_pixel_pos,intensity))



class Beacons:
    #Beacon params
    #dist_threshhold = rospy.get_param('~/beacons/dist_threshhold',2) #in cells
    #dist_between_double = rospy.get_param('~/beacons/dist_between_double',80) #in cells
    #dist_from_double = rospy.get_param('~/beacons/dist_from_double', 155) #in cells (median of base of the triangle)
    #############
    raw_list = []
    raw_list.append(rospy.get_param('~/beacons/beacon1',[154,2])) #in cells, first beacon is top-left, then - counterclockwise
    raw_list.append(rospy.get_param('~/beacons/beacon2',[-3,50])) #in cells
    raw_list.append(rospy.get_param('~/beacons/beacon3',[154,99])) #in cells
    #############
    
    #/Beacon params
    new_coords = []
    list = []
    pose = (0,0)
    def __init__(self,pos:list):
        self.pose = (pos[0], pos[1])
        Beacons.list.append(self)
    @classmethod
    def reInit(cls):
        cls.list.clear()
        for pose in cls.raw_list:
            beacon = cls(pose) #initialisation auto-appends objects

def main():
    rate = rospy.Rate(Laser.update_rate)
    Beacons.reInit()
    while not rospy.is_shutdown():




        rate.sleep()


    
if __name__ == '__main__':
    main()