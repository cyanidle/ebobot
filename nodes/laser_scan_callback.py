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
    skip_scans = rospy.get_param("~skipped_scans",0)  #number of skipped per update
    skipped_counter = 0
    #/Features
    #Params
    #
    min_dots = rospy.get_param('~min_dots', 2)
    minimal_x = rospy.get_param('~minimal_x', -0.2)
    maximum_x = rospy.get_param('~maximum_x', 2.4)
    minimal_y = rospy.get_param('~minimal_y ', -0.2)
    maximum_y = rospy.get_param('~maximum_y', 3.4)
    dist_between_dots_minimal = rospy.get_param('~dist_between_dots_minimal', 0.05) #in meters
    #
    update_rate = rospy.get_param("~update_rate",2) #updates/sec
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
            Laser.coeffs.append((sin(Laser.angle_increment*num),cos(Laser.angle_increment*num))) 
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
        new_list = []
        cls.updateTF()
        for range, intensity, coeffs in zip(cls.ranges, cls.intensities, cls.coeffs):
            y_coeff, x_coeff = coeffs
            if range < cls.range_max and range > cls.range_min:
                meters_pos = np.array((range * y_coeff, range * x_coeff))          
                prob_meters_pos = np.array(turnVect(meters_pos,  -cls.robot_pos[2] + cls.rads_offset)) + cls.robot_pos[:2]
                if (cls.minimal_x < prob_meters_pos[1] < cls.maximum_x
                 and cls.minimal_y < prob_meters_pos[0] < cls.maximum_y):
                    new_list.append((prob_meters_pos,intensity))
        cls.list = new_list
    @classmethod
    def find(cls): 
        curr_obst = []
        if len(Laser.list)>2:
            curr_obst.append(Laser.list[0][0])
            Beacons.clearRelative()
            Objects.clear()
            last_num = 0
            for scan in Laser.list[1:]:
                pose, intencity = scan
                try:
                    dist = np.linalg.norm((pose[0] - Laser.list[last_num][0][0] ,  pose[1] - Laser.list[last_num][0][1]))
                except:
                    #pass
                    # print(f"Error! {Laser.list = }")
                    # print(f"{len(Laser.list) = }")
                    # print(f"{last_num = }")
                    rospy.logwarn(f"Laser error!")
                    break
                last_num = last_num +1
                if dist<cls.dist_between_dots_minimal:
                    curr_obst.append(pose)
                elif len(curr_obst) >= cls.min_dots:
                    radius = np.linalg.norm(
                        (curr_obst[0][0] - curr_obst[-1][0],  curr_obst[0][1] - curr_obst[-1][1]    ))
                    if Beacons.min_dots < len(curr_obst) < Beacons.dots_thresh:
                        pos = cls.getPosition(curr_obst)
                        for exp in Beacons.expected_list:
                            if np.linalg.norm(( pos[0] - exp.pose[0]  ,pos[1] - exp.pose[1] )):
                                Beacons(pos,exp.num)
                    elif Objects.min_dots < len(curr_obst) < Objects.dots_thresh:
                        if radius < Objects.safe_footprint_radius:
                            radius = Objects.safe_footprint_radius
                        Objects(cls.getPosition(curr_obst), radius*Objects.radius_coeff)
                    curr_obst.clear()
    @classmethod
    def getPosition(cls,poses):
        "(Laser) Simply returns algebraic median from list of positions, may get an upgrade later"
        x, y = 0,0
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
    enable_adjust = rospy.get_param('~beacons/enable_adjust', 0)
    # /Features
    ###
    max_dist_from_expected = rospy.get_param('~beacons/max_dist_from_expected', 0.3) #in meters
    min_dots = rospy.get_param('~beacons/min_dots', 5)
    dots_thresh = rospy.get_param('~beacons/dots_thresh', 15) #num
    #############
    raw_list = []
    raw_list.append(rospy.get_param('~beacons/beacon1',[154*0.02,99*0.02])) #in meters, first beacon is top-left, then - counterclockwise
    raw_list.append(rospy.get_param('~beacons/beacon2',[154*0.02,2*0.02])) #in meters
    raw_list.append(rospy.get_param('~beacons/beacon3',[-3*0.02,50*0.02])) #in meters
    #############
    #/Beacon params
    #Globals
    delta_th = 0
    delta_pos = (0,0)
    pose = (0,0)
    expected_list = []
    rel_list = []
    #/Globals
    def __init__(self,pos:tuple,num:int = 10,expected:int = 0):
        self.num = num
        self.pose = (pos[0], pos[1])
        if expected:
            Beacons.expected_list.append(self)
        else:
            Beacons.rel_list.append(self)
    def __sub__(self,other):
        return (   self.pose[0] - other.pose[0]     ,    self.pose[1] - other.pose[1]      )      
    def __truediv__(self,other):
        return (   self.pose[0] - other.pose[0]     ,    self.pose[1] - other.pose[1]      ) 
    @classmethod
    def initExpected(cls):
        for num,coord in enumerate(cls.raw_list):
            new_beacon = cls(coord,num,expected = 1)
    # @classmethod
    # def initRelative(cls, pose,num):   
    #     beacon = cls(pose,num) #initialisation auto-appends objects to their list
    @classmethod
    def pubRelative(cls):
        for num,beacon in enumerate(cls.rel_list):
            rel_pos = (beacon.pose[0],  beacon.pose[1])
            pubMarker(rel_pos,num,1/cls.update_rate,frame_name="relative_beacon",type="cylinder",size=0.12,g=1,r=1,b=1,debug=Laser.debug,add=1)
    @classmethod
    def clearRelative(cls):
        cls.rel_list.clear()
    @classmethod
    def update(cls):
        "The most important func in localisation"
        cls.pubRelative()
        exp_list = cls.expected_list
        rel_list = cls.rel_list
        if len(rel_list) < 2:
            cls.rel_list.clear()
        else:
            rel_line = []
            exp_line = []
            rel_line= ((rel_list[1].pose[0] - rel_list[0].pose[0],    rel_list[1].pose[1] - rel_list[0].pose[1] ))
            exp_line = ((exp_list[1].pose[0] - exp_list[0].pose[0],    exp_list[1].pose[1] - exp_list[0].pose[1] ))
            cls.delta_th = atan(   (rel_line[1]-exp_line[1]) / (rel_line[1]-exp_line[1])  ) #try changing the order of division and sign if fails
            rel_line = turnVect( rel_line, cls.delta_th) #turn both beacons
            d_x, d_y = 0, 0
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
    safe_footprint_radius = rospy.get_param('~obstacles/safe_footprint_radius', 0.2)
    radius_coeff = rospy.get_param('~obstacles/radius_coeff', 1.2)
    min_dots = rospy.get_param('~obstacles/min_dots', 20)
    dots_thresh = rospy.get_param('~obstacles/dots_thresh', 200) #num
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
            #pubMarker(obst.pose,num,1/Laser.update_rate,frame_name="objects",type="cube",size=0.1,g=0.5,r=1,b=0.5,debug=Laser.debug,add=1)
            obstacle.y = obst.pose[0] 
            obstacle.x = obst.pose[1]
            obstacle.radius = obst.radius #PLACEHOLDER
            msg.data.append(obstacle)
        cls.list_pub.publish(msg)
    @classmethod
    def clear(cls):
        cls.list.clear()
#################################################################
def shutdownHook():
    msg = Obstacles()
    obstacle = Obstacle()
    msg.data.append(obstacle)
    Objects.list_pub.publish(msg)
def main():
    rate = rospy.Rate(Laser.update_rate)
    rospy.on_shutdown(shutdownHook)
    Beacons.initExpected()
    while not rospy.is_shutdown():
        Laser.find()
        Beacons.update()
        Objects.send()
        rate.sleep()


    
if __name__ == '__main__':
    main()
