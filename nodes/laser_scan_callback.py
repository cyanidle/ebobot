#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import numpy as np
import rospy
import tf
from dorlib import dCoordsInRad
rospy.init_node('laser_scan_callback')
def robotPosCallback(odom):
    quat = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    Laser.robot_pos = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y,tf.transformations.euler_from_quaternion(quat)[2]]) / Laser.costmap_resolution
class Laser:
    #Params
    #/Params



    #Global values
    robot_pos = np.array([0,0,0])
    #/Global values
    