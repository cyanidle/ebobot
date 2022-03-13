#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import numpy as np
from math import cos,sin,atan#,atan2
import rospy
import tf
#####################
from dorlib import turnVect
from markers import pubMarker#, transform
from ebobot.msg import Obstacles, Obstacle
######################
from nav_masgs.msg import Odometry, OccupancyGrid, Path, PoseStamped
from sensor_msgs.msg import LaserScan
######################
rospy.init_node('laser_scan_callback')




###############


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
    
    
    dist_dots_threshhold = rospy.get_param('~dist_dots_threshhold', 0.1) #in meters
    #
    update_rate = rospy.get_param("~update_rate",5) #updates/sec
    rads_offset = rospy.get_param("~rads_offset",1.5) #in radians diff from lidar`s 0 rads and costmap`s in default position(depends where lidars each scan starts, counterclockwise)
    costmap_resolution = rospy.get_param("~costmap_resolution",0.02) #meters/cell
    #/Params

    #Topics
    costmap_topic = rospy.get_param('~costmap_topic','/costmap')
    laser_scan_topic = rospy.get_param("~laser_scan_topic", "/laser_scan")
    robot_pos_topic = rospy.get_param("~robot_pos_topic", "/odom")
    robot_pos_adj_topic = rospy.get_param("~robot_pos_adj_topic", "/initialpose")
    #
    broadcaster = tf.TransformBroadcaster()
    adjust_publisher = rospy.Publisher(robot_pos_adj_topic, Odometry, queue_size = 5)
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
        quat = tf.transformations.quaternion_from_euler(0,0,-cls.robot_pos[2]-cls.rads_offset)
        cls.laser_broadcaster.sendTransform(
            (cls.robot_pos[1], cls.robot_pos[0], 0),
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
                prob_meters_pos = np.array(turnVect(meters_pos - cls.robot_pos[:2],  -cls.robot_pos[2] + cls.rads_offset))
                cls.new_list.append((prob_meters_pos,intensity))
        cls.list = cls.new_list
        cls.findRelative()
    @classmethod
    def findRelative(cls): #todo: move to laser, check cost
        curr_obst = []
        curr_obst.append(Laser.list[0][0])
        Beacons.clearRelative()
        Objects.clear()
        for prev_num, scan in enumerate(Laser.list[1:]):
            pose, intencity = scan
            #x,y = pose
            dist = np.linalg.norm(pose - Laser.list[prev_num])
            if dist<cls.dist_dots_threshhold:
                curr_obst.append(pose)
            else:
                if len(curr_obst) < Beacons.dots_thresh:
                    Beacons.initRelative(cls.getPosition(curr_obst))
                elif len(curr_obst) < Objects.dots_thresh:
                    Objects(cls.getPosition(curr_obst))
                curr_obst.clear()
        #Obstacles.send()
    @classmethod
    def getPosition(cls,poses):
        "(Laser) Simply returns algebraic median from list of positions, may get an upgrade later"
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
    raw_list.append(rospy.get_param('~/beacons/beacon1',[154*0.02,2*0.02])) #in meters, first beacon is top-left, then - counterclockwise
    raw_list.append(rospy.get_param('~/beacons/beacon2',[-3*0.02,50*0.02])) #in meters
    raw_list.append(rospy.get_param('~/beacons/beacon3',[154*0.02,99*0.02])) #in meters
    #############
    
    
    #/Beacon params

    #Globals
    delta_th = 0
    delta_pos = (0,0)
    pose = (0,0)
    expected_list = []
    rel_list = []
    #/Globals

    def __init__(self,pos:tuple,expected:int = 0):
        self.pose = (pos[0], pos[1])
        if expected:
            Beacons.expected_list.append(self)
            
        else:
            Beacons.rel_list.append(self)


    @classmethod
    def initExpected(cls,raw_list):
        for num,coord in enumerate(raw_list):
            pos = cls.getPosition(coord)
            new_beacon = cls(pos,1)
            pubMarker(pos,num,0,frame_name="expected_beacon",type="cylinder",duration=0,size=0.12,g=0,r=1,debug=Laser.debug,add=1)
        #cls.rel_list.append(new_beacon)
    @classmethod
    def initRelative(cls, pose):   
        beacon = cls(pose) #initialisation auto-appends objects to their list
        pubMarker(pose,1,1,frame_name="first_found_beacon",type="cylinder",duration=0.4,size=0.12,g=1,r=1,b=1,debug=Laser.debug,add=1)
            
    @classmethod
    def getExpected(cls):
        rel_poses = []
        pubMarker(rel_pos,num,1,frame_name="relative_beacon",type="cylinder",duration=0,size=0.12,g=0,r=1,b=1,debug=Laser.debug,deletall=1)
        for num,beacon in enumerate(cls.expected_list):
            rel_pos = turnVect((beacon.pose[0]- Laser.robot_pos[0], beacon.pose[1] - Laser.robot_pos[1]), - Laser.robot_pos[2])
            rel_poses.append(rel_pos)  
            pubMarker(rel_pos,num,1,frame_name="relative_beacon",type="cylinder",duration=0,size=0.12,g=0,r=1,b=1,debug=Laser.debug,add=1)
        return rel_poses
    @classmethod
    def clearRelative(cls):
        cls.rel_list.clear()
    @staticmethod
    def rearrangeExpList(exp_list, rel_list):
        "ITS IMPORTANT to pass relative list second"
        new_exp_list = ['empty' for _ in range(len(exp_list))]
        for rel_beacon in rel_list:
            min_dist = 100 #should be more than any dist
            min_num = 0
            for num,exp_beacon in enumerate(exp_list):
                dist = np.linalg.norm(rel_beacon.pose - exp_beacon.pose)
                if dist < min_dist:
                    min_num = num
                    min_dist = dist
            new_exp_list[min_num] = rel_beacon
        new_exp_list[:] = [pos for pos in new_exp_list if pos != 'empty']
        return new_exp_list
    @classmethod
    def update(cls):
        "The most importatnt func in localisation"
        exp_list = np.array(cls.getExpected())
        rel_list = np.array(cls.rel_list)
        exp_list = cls.rearrangeExpList(exp_list, rel_list)
        if len(rel_list) < 2:
            rospy.logwarn("Less than 2 beacons found")
            return
        else:
            exp, rel =exp_list[:2],rel_list[:2] #cuts off the first two beacons
            rel_line = np.array(     (rel[0][0]-rel[1][0],    rel[0][1]-rel[1][1])   )
            exp_line = np.array(     (exp[0][0]-exp[1][0],    exp[0][1]-exp[1][1])   )
            cls.delta_th = atan(   (rel_line[1]-exp_line[1]) / (rel_line[0]-exp_line[0])  ) #try changing the order of division and sign if fails
            rel = map(turnVect, rel, [cls.delta_th]*2) #turn both beacons
            # get d_x adn d_y
            d_x = d_y = 0
            for i in range(2):
                d_y += rel[i][0] - exp[i][0]
                d_x += rel[i][1] - exp[i][1]
            d_y = d_y/2
            d_x = d_x/2
            #
            cls.delta_pos = (d_y,d_x)
            cls.publishAdjust()
            
    @classmethod
    def publishAdjust(cls):
        new = PoseStamped()
        new.header.stamp = rospy.Time.now()
        new.header.frame_id = "odom"
        new_quat = tf.transformations.quaternion_from_euler(0, 0, cls.robot_pos[2] + cls.delta_th)
        new.pose.position.x = cls.robot_pos[1] + cls.delta_pos[1]
        new.pose.position.y = cls.robot_pos[0] + cls.delta_pos[0]
        new.pose.orientation.x = new_quat[0]
        new.pose.orientation.y = new_quat[1]
        new.pose.orientation.z = new_quat[2]
        new.pose.orientation.w = new_quat[3]
        cls.adjust_publisher.publish(new)

       
#################################################################        
    



#################################################################
class Objects(Laser):

    #Params
    dots_thresh = rospy.get_param('~/obstacles/dots_thresh', 10) #num
    #/Params

    #Topics
    list_topic = rospy.get_param('~/obstacles/list_topic', Obstacles ,'obstacles')
    #
    list_pub = rospy.Publisher(list_topic,Path,queue_size = 2)
    #/Topics

    list = []
    def __init__(self, dots_list:list):
        self.pose = Obstacles.getPosition(dots_list)
        Obstacles.list.append(self)
    
    @classmethod
    def send(cls):
        msg = Obstacles()
        #msg.header.frame_id = "obstacles"
        #msg.header.stamp = rospy.Time.now()
        for obst in cls.list:
            #cls.
            obstacle = Obstacle()
            obstacle.y = obst.pose[0] 
            obstacle.x = obst.pose[1]
            obstacle.size = 0 #PLACEHOLDER
            msg.data.append(obstacle)
        cls.list_pub.publish(msg)
    @classmethod
    def clear(cls):
        cls.list.clear()
#################################################################

def main():
    rate = rospy.Rate(Laser.update_rate)
    #Beacons.initExpected()
    while not rospy.is_shutdown():
        Beacons.update()
        Objects.send()
        rate.sleep()


    
if __name__ == '__main__':
    main()
