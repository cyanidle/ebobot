#!/usr/bin/env python

import roslib
roslib.load_manifest('ebobot')
import rospy
#import math

import tf

rospy.init_node('costmap_server')
##########
from nav_msgs.msg import OccupancyGrid, OccupancyGridUpdate
from map_msgs.msg import OccupancyGridUpdate
###########
import png
from libraries.DorLib import deltaCoordsInRad


class Costmap():
    #Params
    file = rospy.get_param('costmap_server/file','/src/costmap.png')
    #/Params




    #Global
    list = []
    #/Global
    file = png.Reader('src/costmap.png')
    width,height,pixels,metadata = file.read()

    def __init__(self):
        Costmap.list.append(self)
    def initCostmap(self):
        raw_costmap = [[Costmap.pixels[x+y] for x in range(Costmap.height)] for y in range(Costmap.width)]


    def publishUpdate(self): ###An example
        msg = OccupancyGridUpdate()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.width
        msg.height
        for goal,_ in Global.list:
                msg.data[i]
        goal.path_publisher.publish(msg)
        rospy.loginfo(f"Published new route with {len(Global.list)} points")   
    def publish(self): ###An example
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



if __name__=="__main__":
    
