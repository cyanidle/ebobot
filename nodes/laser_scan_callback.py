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
        cls.list.clear()
        cls.updateTF()
        for range, intensity, coeffs in zip(cls.ranges, cls.intensities, cls.coeffs):
            y_coeff, x_coeff = coeffs
            if range < cls.range_max and range > cls.range_min:
                meters_pos = np.array((range * y_coeff, range * x_coeff))
                #abs_pixel_pos = meters_pos/cls.costmap_resolution
                prob_meters_pos = np.array(turnVect(meters_pos - cls.robot_pos[:2],  -cls.robot_pos[2]))
                #cls.abs_list.append((meters_pos,intensity))
                cls.list.append((prob_meters_pos,intensity))



class Beacons:
    #Beacon params
    
    #############
    raw_list = []
    raw_list.append(rospy.get_param('~/beacons/beacon1',[154*2,2*2])) #in meters, first beacon is top-left, then - counterclockwise
    raw_list.append(rospy.get_param('~/beacons/beacon2',[-3*2,50*2])) #in meters
    raw_list.append(rospy.get_param('~/beacons/beacon3',[154*2,99*2])) #in meters
    #############
    dist_dots_threshhold = rospy.get_param('~/beacons/cost_check_radius', 8) #in meters
    
    #/Beacon params

    #Globals
    expected_list = []
    list = []
    #cost_coords_list = dCoordsInRad(cost_check_radius,cost_check_resolution)
    #new_list =  []
    #pose = (0,0)
    #/Globals

    def __init__(self,pos:list,expected:int = 0):
        self.pose = (pos[0], pos[1])
        if expected:
            Beacons.expected_list.append(self)
        else:
            Beacons.list.append(self)


    @classmethod
    def initExpected(cls):
        for pose in cls.raw_list:
            beacon = cls(pose,expected = 1)
    @classmethod
    def initRelative(cls, new_list:list):
        cls.list.clear()
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
    def findRelative(cls):
        list = []
        curr_obst = []
        curr_obst.append(Laser.list[0][0])
        for prev_num, scan in enumerate(Laser.list[1:]):
            pose, intencity = scan
            #x,y = pose
            dist = np.linalg.norm(pose - Laser.list[prev_num])
            if dist<cls.dist_dots_threshhold:
                curr_obst.append(pose)
            else:
                Obstacles(curr_obst)
                curr_obst.clear()


        
        return list
        #pass
    @classmethod
    def update(cls):
        cls.initRelative(cls.findRelative())
        exp_list = cls.getExpected()
        
        pass
    
class Obstacles:

    #Params

    #/Params

    #Topics
    list_topic = rospy.get_param('~/obstacles/list_topic', Path ,'obstacles')
    #
    list_pub = rospy.Publisher(list_topic,LaserScan,laserScanCallback)
    #/Topics

    list = []
    def __init__(self, poses_list:list):
        self.pose = Obstacles.getPosition(poses_list)
        Obstacles.list.append(self)
    def getPosition(self,poses:list):
        pass
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



def main():
    rate = rospy.Rate(Laser.update_rate)
    Beacons.initExpected()
    while not rospy.is_shutdown():
        Beacons.update()



        rate.sleep()


    
if __name__ == '__main__':
    main()