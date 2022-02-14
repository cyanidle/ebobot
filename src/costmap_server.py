#!/usr/bin/env python
import roslib
roslib.load_manifest('ebobot')
import rospy
#import math
import cmath
import tf
import numpy as np
rospy.init_node('costmap_server')
##########
from nav_msgs.msg import OccupancyGrid, OccupancyGridUpdate
from map_msgs.msg import OccupancyGridUpdate
###########
import png


def publish(): ###An example
        msg = Path()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        for goal,_ in Global.list:
                pose = PoseStamped()
                pose.pose.position.x = goal.vect[0]
                pose.pose.position.y = goal.vect[1]
                pose.pose.position.z = 0
                quaternion = tf.transformations.quaternion_from_euler(0, 0, goal.vect[2])
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                msg.poses.append(pose) 
        goal.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Global.list)} points") 
        