#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import numpy as np
from math import cos,sin,acos#,atan2
import rospy
import tf
#####################
from dorlib import applyRotor, getRotor, turnVect
from markers import pubMarker#, transform
from ebobot.msg import Obstacles, Obstacle
######################
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
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
        rotor = getRotor(-cls.robot_pos[2] + cls.rads_offset)
        for range, intensity, coeffs in zip(cls.ranges, cls.intensities, cls.coeffs):
            y_coeff, x_coeff = coeffs
            if range < cls.range_max and range > cls.range_min:
                meters_pos = (range * y_coeff, range * x_coeff) 
                rotated_meters_pos = applyRotor(meters_pos,  rotor)
                prob_meters_pos =  (rotated_meters_pos[0]+ cls.robot_pos[0],  rotated_meters_pos[1]+ cls.robot_pos[1])
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
            for scan, last_scan in zip(Laser.list[1:],Laser.list[:-1]):
                pose, intencity = scan
                last_pose, last_intencity = last_scan
                try:
                    dist = np.linalg.norm((pose[0] - last_pose[0] ,  pose[1] - last_pose[1]))
                except:
                    rospy.logwarn(f"Misha lox (ya nakosyachil)!")
                    break
                if dist<cls.dist_between_dots_minimal:
                    curr_obst.append(pose)
                elif len(curr_obst) >= cls.min_dots:
                    radius = np.linalg.norm(
                        (curr_obst[0][0] - curr_obst[-1][0],  curr_obst[0][1] - curr_obst[-1][1]    ))
                    pos = cls.getPosition(curr_obst)
                    #rospy.loginfo(f"Found something at {pos} {radius = }")
                    if Beacons.min_rad < radius < Beacons.max_rad:
                        #beacon_dist = 
                        #rospy.loginfo(f"Potential beacon at {pos}, {radius = }")
                        for exp in Beacons.expected_list:
                            beacon_dist = np.linalg.norm(( pos[0] - exp.pose[0]  ,pos[1] - exp.pose[1] ))
                            if beacon_dist < Beacons.max_dist_from_expected:
                                Beacons(pos,exp.num)
                    if  (Objects.minimal_x < pos[1] < Objects.maximum_x and
                        Objects.minimal_y < pos[0] < Objects.maximum_y
                        and Objects.min_dots < len(curr_obst) < Objects.dots_thresh):
                        if radius < Objects.safe_footprint_radius:
                            radius = Objects.safe_footprint_radius
                        Objects(pos, radius*Objects.radius_coeff)
                    curr_obst.clear()
    @classmethod
    def getPosition(cls,poses):
        "(from class Laser) Simply returns algebraic median from list of positions, may get an upgrade later"
        x, y = 0,0
        for pos in poses:
            y += pos[0]
            x += pos[1]
        max = len(poses)
        return (y/max,x/max)
#################################################################
def adjCB(req):
        Beacons._adjust_flag = 1
        rospy.sleep(Beacons.adjust_time)
        Beacons._adjust_flag = 0
        Beacons.cycle = 1
        Beacons.deltas.clear()
        return EmptyResponse()
class Beacons(Laser):
    #Beacon params
    # Features
    only_linear_adj = rospy.get_param('~beacons/only_linear_adj', 0)
    switching_adjust = rospy.get_param('~beacons/switching_adjust', 0)#do not use
    enable_adjust = rospy.get_param('~beacons/enable_adjust', 1)
    adjust_on_command = rospy.get_param('~beacons/adjust_on_command', 0)
    pub_all = rospy.get_param('~beacons/pub_all', 1)
    # /Features
    ###
    max_th_for_linear_adj = rospy.get_param('~beacons/max_th_for_linear_adj', 0.01)
    max_th_adj = rospy.get_param('~beacons/max_th_adj', 0.5) #radians
    max_pos_adj = rospy.get_param('~beacons/max_pos_adj', 0.5) #meters
    kostyl = rospy.get_param('~beacons/kostyl', 0.005)
    adjust_time = rospy.get_param('~beacons/adjust_time', 2) #seconds for adjustment
    cycles_per_update = rospy.get_param('~beacons/cycles_per_update', 2)
    max_dist_from_expected = rospy.get_param('~beacons/max_dist_from_expected', 0.3) #in meters
    min_rad = rospy.get_param('~beacons/min_rad', 0.01) #meters
    max_rad = rospy.get_param('~beacons/max_rad', 0.2) #meters
    ############# 
    raw_list = []
    # Auto-init beacons 
    num_beacons = rospy.get_param('~beacons/num_beacons', 3)
    for n in range(num_beacons):
        raw_list.append(rospy.get_param(f'~beacons/beacon{n}'))
    #
    ############# Manual init
    # raw_list.append(rospy.get_param('~beacons/beacon0',[0.02,1])) #in meters, first beacon is top-left, then - counterclockwise
    # raw_list.append(rospy.get_param('~beacons/beacon1',[2.99,0.01])) #in meters
    # raw_list.append(rospy.get_param('~beacons/beacon2',[2.99,1.99])) #in meters
    # raw_list.append(rospy.get_param('~beacons/test_beacon0',[1,0.7])) #in meters
    # raw_list.append(rospy.get_param('~beacons/test_beacon1',[1,1.3])) #in meters
    # raw_list.append(rospy.get_param('~beacons/test_beacon2',[0,1])) #in meters
    #############
    #/Beacon params
    #Globals
    deltas = []
    cycle = 0
    delta_th = 0
    delta_pos = (0,0)
    pose = (0,0)
    expected_list = []
    rel_list = []
    _pubbing_rot = 0
    _adjust_flag = not adjust_on_command
    #/Globals
    if adjust_on_command:
        rospy.Service("adjust_pos_service", Empty, adjCB)
    def __init__(self,pos:tuple,num:int = 10,expected:int = 0):
        self.num = num
        self.pose = (pos[0], pos[1])
        if expected:
            Beacons.expected_list.append(self)
        else:
            self._pub = False
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
    def pubRelative(cls, _all = False):
        for num,beacon in enumerate(cls.rel_list):
            if beacon._pub or _all:
                rel_pos = (beacon.pose[0],  beacon.pose[1])
                pubMarker(rel_pos,num,1/cls.update_rate,frame_name="relative_beacon",type="cylinder",height=0.4,size=0.1,g=0.5,r=1,b=0.5,debug=Laser.debug,add=1)
    @classmethod
    def pubExpected(cls):
        for num,beacon in enumerate(cls.expected_list):
            exp_pos = (beacon.pose[0],  beacon.pose[1])
            pubMarker(exp_pos,num,1/cls.update_rate,frame_name="expected_beacon",type="cylinder",height=0.35,size=0.1,g=1,r=1,b=1,debug=Laser.debug,add=1)
    @classmethod
    def clearRelative(cls):
        cls.rel_list.clear()
    @classmethod
    def rearrangeRels(cls,rels):
        return 
    @classmethod
    def update(cls):
        "The most important func in localisation"
        cls.pubExpected()
        if len(cls.rel_list) < 2:
            cls.rel_list.clear()
            cls.cycle = 0
            cls.deltas.clear()
        elif cls._adjust_flag:
            exp_list = [] 
            rel_list = []
            _rel_list_meta = []
            nums = []
            min_dists = [100,100,100]
            new_rels = sorted(cls.rel_list, key= lambda _rel: _rel.num)
            for rel in new_rels:
                if not rel.num in nums and len(nums) < 2:
                    nums.append(rel.num)    
                if rel.num in nums:
                    _curr_rel_dist = np.linalg.norm(rel - cls.expected_list[rel.num])
                    if _curr_rel_dist < min_dists[rel.num]:
                        rel._pub = True
                        min_dists[rel.num] = _curr_rel_dist
                        if len(rel_list) < 2 and not rel.num in _rel_list_meta:
                            rel_list.append(rel)
                            _rel_list_meta.append(rel.num) #remaps number to the list
                        else: 
                            rel_list[_rel_list_meta.index(rel.num)] = rel
            cls.pubRelative(_all=cls.pub_all)
            if len(nums) <2:
                return
            for num in nums:
                exp_list.append(cls.expected_list[num]) #this parts sets up two beacons
            rel_line = np.array((rel_list[1].pose[0] - rel_list[0].pose[0],    rel_list[1].pose[1] - rel_list[0].pose[1] ))
            exp_line = np.array((exp_list[1].pose[0] - exp_list[0].pose[0],    exp_list[1].pose[1] - exp_list[0].pose[1] ))
            ##################################### Даже не спрашивайте...
            delta_exp = acos(exp_line[1]/np.linalg.norm(exp_line))
            if (np.array(turnVect(exp_line,-delta_exp))[0] < cls.kostyl):
                delta_exp =  -delta_exp
            delta_rel = acos(rel_line[1]/np.linalg.norm(rel_line))
            if (np.array(turnVect(rel_line,-delta_rel))[0] < cls.kostyl):
                delta_rel =  -delta_rel
            delta_th =   -(delta_exp - delta_rel)
            #####################################
            if (abs(delta_th) > cls.max_th_for_linear_adj 
            and not cls.only_linear_adj):
                rospy.logwarn_once(f"Change lines 320-322 if fails")
                _curr_adj = np.array(exp_list[0].pose) - np.array(turnVect(rel_list[0].pose- cls.robot_pos[:2], -delta_th)) + cls.robot_pos[:2] 
                _curr_adj += np.array(exp_list[1].pose) - np.array(turnVect(rel_list[1].pose- cls.robot_pos[:2], -delta_th)) + cls.robot_pos[:2] 
                _curr_adj /= 2
            else:
                _curr_adj = np.array(exp_list[0] - rel_list[0])
                _curr_adj = (_curr_adj+np.array(exp_list[1] - rel_list[1]))/2
            ####################################
            d_x,d_y = _curr_adj[1],_curr_adj[0]
            if Laser.debug:
                rospy.logwarn(f"{d_x = } , {d_y = }, {delta_th = }")
            cls.deltas.append((d_y, d_x, delta_th))
            cls.checkCycle()
            cls.rel_list.clear()
    @classmethod
    def checkCycle(cls):
        if cls.cycle >= cls.cycles_per_update and cls.enable_adjust and len(cls.deltas):
            _sum_x,_sum_y, _sum_th = 0,0,0
            for _y, _x, _th in cls.deltas:
                _sum_x += _x
                _sum_y += _y
                _sum_th += _th
            _max = len(cls.deltas)
            cls.delta_th = _sum_th/_max
            cls.delta_pos = (_sum_y/_max,_sum_x/_max)
            if (not -cls.max_th_adj < cls.delta_th < cls.max_th_adj
             or np.linalg.norm(cls.delta_pos) > cls.max_pos_adj):
                cls.delta_th = 0
                cls.delta_pos = (0,0)
                rospy.logerr("Adjustment error!")
            if cls.debug:
                rospy.logerr(f"{cls.delta_pos = }|{cls.delta_th = } ")
            if cls.switching_adjust:
                if cls._pubbing_rot:
                    cls._pubbing_rot = 0
                    cls.delta_pos = (0,0)
                else:
                    cls._pubbing_rot = 1
                    cls.delta_th = 0
            cls.publishAdjust()
            cls.cycle = 1
            cls.deltas.clear()
            #raise SyntaxError("Epic")
        else:
            cls.cycle += 1
               
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
    min_dots = rospy.get_param('~obstacles/min_dots', 4)
    dots_thresh = rospy.get_param('~obstacles/dots_thresh', 140) #num
    #
    minimal_x = rospy.get_param('~obstacles/minimal_x', 0.05)
    maximum_x = rospy.get_param('~obstacles/maximum_x', 2)
    minimal_y = rospy.get_param('~obstacles/minimal_y ', 0.05)
    maximum_y = rospy.get_param('~obstacles/maximum_y', 3)
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
