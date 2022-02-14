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



def publishUpdate(): ###An example
        msg = OccupancyGridUpdate()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.width
        msg.height
        for goal,_ in Global.list:
                msg.data[i]
        goal.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Global.list)} points") 
        
def publish(): ###An example
        msg = OccupancyGrid()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution
        msg.info.width
        msg.info.height
        for goal,_ in Global.list:
                msg.data[i]
        goal.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Global.list)} points") 
        