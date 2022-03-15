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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry#, OccupancyGrid
from sensor_msgs.msg import LaserScan
######################
rospy.init_node('laser_scan_callback')




###############


def robotPosCallback(odom):
    quat = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    Laser.robot_pos = np.array([odom.pose.pose.position.y,odom.pose.pose.position.x,tf.transformations.euler_from_quaternion(quat)[2]]) 

# def costmapCallback(costmap):
#     Laser.costmap_resolution = costmap.info.resolution
#     Laser.costmap_height = costmap.info.height
#     Laser.costmap_width= costmap.info.width
#     rospy.loginfo_once(f"Got new map, height = {Laser.costmap_height}, width= {Laser.costmap_width}")
#     Laser.costmap= np.reshape(costmap.data,(Laser.costmap_height, Laser.costmap_width))  
#     #Laser.debug_map = Laser.costmap
def laserScanCallback(scan):
    if Laser.skipped_counter < Laser.skip_scans:
        Laser.skipped_counter += 1
    else:
        Laser.skipped_counter = 0
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
    debug = rospy.get_param("~debug",1)
    skip_scans = rospy.get_param("~skipped_scans",1)  #number of skipped per update
    skipped_counter = 0
    #/Features
    #Params
    #
    minimal_x = rospy.get_param('~minimal_x', -0.1)
    maximum_x = rospy.get_param('~maximum_x', 2.2)
    minimal_y = rospy.get_param('~minimal_y ', -0.1)
    maximum_y = rospy.get_param('~maximum_y', 3.2)
    dist_dots_threshhold = rospy.get_param('~dist_dots_threshhold', 0.05) #in meters
    #
    update_rate = rospy.get_param("~update_rate",1) #updates/sec
    rads_offset = rospy.get_param("~rads_offset",0) #in radians diff from lidar`s 0 rads and costmap`s in default position(depends where lidars each scan starts, counterclockwise)
    #/Params
    #Topics
    laser_scan_topic = rospy.get_param("~laser_scan_topic", "/scan")
    robot_pos_topic = rospy.get_param("~robot_pos_topic", "/odom")
    robot_pos_adj_topic = rospy.get_param("~robot_pos_adj_topic", "/initialpose")
    #
    broadcaster = tf.TransformBroadcaster()
    adjust_publisher = rospy.Publisher(robot_pos_adj_topic, PoseWithCovarianceStamped, queue_size = 5)
    laser_scan_subscriber = rospy.Subscriber(laser_scan_topic,LaserScan,laserScanCallback)
    robot_pos_subscriber = rospy.Subscriber(robot_pos_topic,Odometry,robotPosCallback)
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
        for num in range(int(round(Laser.angle_min/cls.angle_increment)),
         int(round(Laser.angle_max/cls.angle_increment))):
            Laser.coeffs.append((cos(Laser.angle_increment*num),sin(Laser.angle_increment*num))) 
        cls.angles_done = 1
    #####################
    @classmethod
    def updateTF(cls):
        #rospy.logerr(f"epic")
        quat = tf.transformations.quaternion_from_euler(0,0, cls.robot_pos[2]+cls.rads_offset)
        cls.broadcaster.sendTransform(
            (cls.robot_pos[1], cls.robot_pos[0], 0),
            quat,
            rospy.Time.now(),
            "laser",
            "costmap" ##???
        )
    ###############
    @classmethod
    def update(cls):
        cls.new_list.clear()
        cls.updateTF()
        for range, intensity, coeffs in zip(cls.ranges, cls.intensities, cls.coeffs):
            y_coeff, x_coeff = coeffs
            if range < cls.range_max and range > cls.range_min:
                meters_pos = np.array((range * y_coeff, range * x_coeff))          
                prob_meters_pos = turnVect(meters_pos + cls.robot_pos[:2],  cls.robot_pos[2] + cls.rads_offset)   
                if (cls.minimal_x < prob_meters_pos[1] < cls.maximum_x
                 and cls.minimal_y < prob_meters_pos[0] < cls.maximum_y):
                    cls.new_list.append((prob_meters_pos,intensity))
        cls.list = cls.new_list
        cls.findRelative()
    @classmethod
    def findRelative(cls): 
        curr_obst = []
        if len(Laser.list):
            curr_obst.append(Laser.list[0][0])
            Beacons.clearRelative()
            Objects.clear()
            for prev_num, scan in enumerate(Laser.list[1:]):
                pose, intencity = scan
                dist = np.linalg.norm(pose - Laser.list[prev_num][0])
                if dist<cls.dist_dots_threshhold:
                    curr_obst.append(pose)
                else:
                    if Laser.debug and len(curr_obst):
                        print(f"Found obstacle! {len(curr_obst) = }, {curr_obst = }")
                    if 0 < len(curr_obst) < Beacons.dots_thresh:
                        Beacons.initRelative(cls.getPosition(curr_obst))
                    elif 0 < len(curr_obst) < Objects.dots_thresh:
                        Objects(cls.getPosition(curr_obst),len(curr_obst)/2)
                    curr_obst.clear()
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
    # Features
    enable_adjust = rospy.get_param('~/beacons/enable_adjust', 0)
    # /Features
    ###
    dots_thresh = rospy.get_param('~/beacons/dots_thresh', 10) #num
    #############
    raw_list = []
    raw_list.append(rospy.get_param('~/beacons/beacon1',[154*0.02,99*0.02])) #in meters, first beacon is top-left, then - counterclockwise
    raw_list.append(rospy.get_param('~/beacons/beacon2',[154*0.02,2*0.02])) #in meters
    raw_list.append(rospy.get_param('~/beacons/beacon3',[-3*0.02,50*0.02])) #in meters
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
    def __sub__(self,other):
        return (   self.pose[0] - other.pose[0]     ,    self.pose[1] - other.pose[1]      )      
    @classmethod
    def initExpected(cls):
        for num,coord in enumerate(cls.raw_list):
            new_beacon = cls(coord,expected = 1)
    @classmethod
    def initRelative(cls, pose):   
        beacon = cls(pose) #initialisation auto-appends objects to their list
    @classmethod
    def getRelative(cls):
        rel_poses = []
        for num,beacon in enumerate(cls.rel_list):
            rel_pos = (beacon.pose[0] - Laser.robot_pos[0],  beacon.pose[1] - Laser.robot_pos[1])
            Beacons(rel_pos)
            pubMarker(rel_pos,num,1,frame_name="relative_beacon",type="cylinder",size=0.12,g=1,r=1,b=1,debug=Laser.debug,add=1)
        return rel_poses
    @classmethod
    def clearRelative(cls):
        cls.rel_list.clear()
    @classmethod
    def update(cls):
        "The most important func in localization"
        exp_list = np.array(cls.expected_list)
        rel_list = np.array(cls.rel_list)
        if len(rel_list) < 2:
            cls.rel_list.clear()
        else:
            rel_line = []
            exp_line = []
            rel_line= ((rel_list[1].pose[0] - rel_list[0].pose[0],    rel_list[1].pose[1] - rel_list[0].pose[1] ))
            exp_line = ((exp_list[1].pose[0] - exp_list[0].pose[0],    exp_list[1].pose[1] - exp_list[0].pose[1] ))
            cls.delta_th = atan(   (rel_line[1]-exp_line[1]) / (rel_line[0]-exp_line[0])  ) #try changing the order of division and sign if fails
            rel_line = turnVect( rel_line, cls.delta_th) #turn both beacons
            d_x = d_y = 0
            for i in range(2):
                d_y += rel_list[i].pose[0] - exp_list[i].pose[0]
                d_x += rel_list[i].pose[1] - exp_list[i].pose[1]
            d_y = d_y/2
            d_x = d_x/2
            #
            cls.delta_pos = (d_y,d_x)
            if cls.enable_adjust:
                cls.publishAdjust()
            cls.rel_list.clear()
    @classmethod
    def publishAdjust(cls):
        new = PoseWithCovarianceStamped()
        new.header.stamp = rospy.Time.now()
        new.header.frame_id = "initialpose"
        new_quat = tf.transformations.quaternion_from_euler(0, 0, cls.robot_pos[2] - cls.delta_th)
        new.pose.pose.position.x = cls.robot_pos[1] + cls.delta_pos[1]
        new.pose.pose.position.y = cls.robot_pos[0] + cls.delta_pos[0]
        new.pose.pose.orientation.x = new_quat[0]
        new.pose.pose.orientation.y = new_quat[1]
        new.pose.pose.orientation.z = new_quat[2]
        new.pose.pose.orientation.w = new_quat[3]
        cls.adjust_publisher.publish(new)
#################################################################
class Objects(Laser):

    #Params
    dots_thresh = rospy.get_param('~obstacles/dots_thresh', 500) #num
    #/Params

    #Topics
    list_topic = rospy.get_param('~obstacles/list_topic' ,'/laser/obstacles')
    #
    list_pub = rospy.Publisher(list_topic,Obstacles,queue_size = 6)
    #/Topics

    list = []
    def __init__(self, pose,radius = 0):
        self.radius = radius
        self.pose = pose
        type(self).list.append(self)
    
    @classmethod
    def send(cls):
        msg = Obstacles()
        for num,obst in enumerate(cls.list):
            obstacle = Obstacle()
            #pubMarker(obst.pose,num,1,frame_name="objects",type="cube",size=0.15,g=0,r=1,b=0,debug=Laser.debug,add=1)
            obstacle.y = obst.pose[0] 
            obstacle.x = obst.pose[1]
            obstacle.radius = obst.radius #PLACEHOLDER
            msg.data.append(obstacle)
        cls.list_pub.publish(msg)
    @classmethod
    def clear(cls):
        cls.list.clear()
#################################################################

def main():
    rate = rospy.Rate(Laser.update_rate)
    Beacons.initExpected()
    while not rospy.is_shutdown():
        Beacons.update()
        Objects.send()
        rate.sleep()


    
if __name__ == '__main__':
    main()
