#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import numpy as np
from math import cos,sin
import rospy
import tf
#####################
from dorlib import dCoordsInRad, turnVect
from markers import pubMarker, transform
######################
from nav_masgs.msg import Odometry, OccupancyGrid, Path, PoseStamped
from sensor_msgs.msg import LaserScan
######################





###############
rospy.init_node('laser_scan_callback')

def robotPosCallback(odom):
    quat = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    Laser.robot_pos = np.array([odom.pose.pose.position.y,odom.pose.pose.position.x,tf.transformations.euler_from_quaternion(quat)[2]]) 
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
    
    
    dist_dots_threshhold = rospy.get_param('~/dist_dots_threshhold', 0.1) #in meters
    #
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
    new_list = []
    list = []
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
        #cls.abs_list.clear()
        cls.new_list.clear()
        cls.updateTF()
        for range, intensity, coeffs in zip(cls.ranges, cls.intensities, cls.coeffs):
            y_coeff, x_coeff = coeffs
            if range < cls.range_max and range > cls.range_min:
                meters_pos = np.array((range * y_coeff, range * x_coeff))
                prob_meters_pos = np.array(turnVect(meters_pos - cls.robot_pos[:2],  -cls.robot_pos[2]))
                cls.new_list.append((prob_meters_pos,intensity))
        cls.list = cls.new_list
        cls.findRelative()
    @classmethod
    def findRelative(cls): #todo: move to laser, check cost
        curr_obst = []
        curr_obst.append(Laser.list[0][0])
        Beacons.clearRelative()
        Obstacles.clear()
        for prev_num, scan in enumerate(Laser.list[1:]):
            pose, intencity = scan
            #x,y = pose
            dist = np.linalg.norm(pose - Laser.list[prev_num])
            if dist<cls.dist_dots_threshhold:
                curr_obst.append(pose)
            else:
                if len(curr_obst) < Beacons.dots_thresh:
                    Beacons.initRelative(cls.getPosition(curr_obst))
                elif len(curr_obst) < Obstacles.dots_thresh:
                    Obstacles(cls.getPosition(curr_obst))
                curr_obst.clear()
    @staticmethod
    def getPosition(poses):
        "Simply returns algebraic median from list of positions, may get an upgrade later"
        x = y = 0
        for pos in poses:
            y += pos[0]
            x += pos[1]
        max = len(poses)
        point = (y/max,x/max)
        return point



#################################################################
class Beacons(Laser):
    #Beacon params
    dots_thresh = rospy.get_param('~/beacons/dots_thresh', 4) #num
    #############
    raw_list = []
    raw_list.append(rospy.get_param('~/beacons/beacon1',[154*2,2*2])) #in meters, first beacon is top-left, then - counterclockwise
    raw_list.append(rospy.get_param('~/beacons/beacon2',[-3*2,50*2])) #in meters
    raw_list.append(rospy.get_param('~/beacons/beacon3',[154*2,99*2])) #in meters
    #############
    
    
    #/Beacon params

    #Globals
    expected_list = []
    rel_list = []
    #/Globals

    def __init__(self,pos:list,expected:int = 0):
        self.pose = (pos[0], pos[1])
        if expected:
            Beacons.expected_list.append(self)
        else:
            Beacons.rel_list.append(self)


    @classmethod
    def initExpected(cls,dots_list):
        new_beacon = cls(cls.getPosition(dots_list),1)
        cls.rel_list.append(new_beacon)
    @classmethod
    def initRelative(cls, new_list:list):
        for pose in new_list:
            beacon = cls(pose) #initialisation auto-appends objects to their list
    @classmethod
    def getExpected(cls):
        rel_poses = []
        for beacon in cls.expected_list:
            rel_pos = turnVect((beacon.pose[0]- Laser.robot_pos[0], beacon.pose[1] - Laser.robot_pos[1]), - Laser.robot_pos[2])
            rel_poses.append(rel_pos)  
        return rel_poses
    @classmethod
    def clearRelative(cls):
        cls.rel_list.clear()
    @classmethod
    def update(cls):
        "The most importatnt func in localisation"
        exp_list = np.array(cls.getExpected())
        rel_list = np.array(cls.rel_list)
        if len(rel_list) < 2:
            return
        else:
            
            pass
#################################################################        
    



#################################################################
class Obstacles(Laser):

    #Params
    dots_thresh = rospy.get_param('~/obstacles/dots_thresh', 10) #num
    #/Params

    #Topics
    list_topic = rospy.get_param('~/obstacles/list_topic', Path ,'obstacles')
    #
    list_pub = rospy.Publisher(list_topic,Path,queue_size = 2)
    #/Topics

    list = []
    def __init__(self, dots_list:list):
        self.pose = Obstacles.getPosition(dots_list)
        Obstacles.list.append(self)
    
    @classmethod
    def send(cls):
        msg = Path()
        msg.header.frame_id = "obstacles"
        msg.header.stamp = rospy.Time.now()
        for obst in cls.list:
            #cls.
            obstacle = PoseStamped()
            obstacle.pose.position.y = obst.pose[0] 
            obstacle.pose.position.x = obst.pose[1]
            obst_quat = tf.transformations.quaternion_from_euler(0, 0, obst.pose[2])
            obstacle.pose.orientation.x = obst_quat[0]
            obstacle.pose.orientation.y = obst_quat[1]
            obstacle.pose.orientation.z = obst_quat[2]
            obstacle.pose.orientation.w = obst_quat[3]
            msg.poses.append(obstacle)
        cls.list_pub.publish(msg)
    @classmethod
    def clear(cls):
        cls.list.clear()
#################################################################

def main():
    rate = rospy.Rate(Laser.update_rate)
    Beacons.initExpected()
    while not rospy.is_shutdown():
        #Beacons.update()



        rate.sleep()


    
if __name__ == '__main__':
    main()
