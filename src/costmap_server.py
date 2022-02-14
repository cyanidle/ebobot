#!/usr/bin/env python

import roslib
roslib.load_manifest('ebobot')
import rospy
#import math
import numpy as np
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
    seconds_per_update = rospy.get_param('costmap_server/seconds_per_update',0.5)
    inflation_radius = rospy.get_param('costmap_server/inflation_radius',0.3)
    inflation_step_radians = rospy.get_param('costmap_server/inflation_step_radians',0.2)
    resolution = rospy.get_param('costmap_server/resolution',0.05)
    file = rospy.get_param('costmap_server/file','/src/costmap.png')

    ##
    costmap_publish_topic = rospy.get_param('costmap_server/costmap_publish_topic','/costmap')
    costmap_update_publish_topic = rospy.get_param('costmap_server/costmap_update_publish_topic','/costmap_update')
    #/Params


    
    #Global
    inflation_coords_list = deltaCoordsInRad(inflation_radius//resolution,inflation_step_radians)
    grid = []
    #list = []
    #/Global
    file = png.Reader('src/costmap.png')
    width,height,pixels,metadata = file.read()
    #[Costmap.pixels[x+y] for x, y in zip(range(Costmap.height),range(Costmap.width))]
    # def __init__(self):
    #     Costmap.list.append(self)
    def initCostmap():
        Costmap.grid = [[Costmap.pixels[x+y] for x in range(Costmap.height+1)] for y in range(Costmap.width+1)]


    def publishUpdate(): ###An example
        msg = OccupancyGridUpdate()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.width
        msg.height
        msg.data
        Costmap.grid_update_publisher.publish(msg)
        
    def publish(): ###An example
        msg = OccupancyGrid()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution = Costmap.resolution
        msg.info.width = Costmap.width
        msg.info.height = Costmap.height
        msg.data = Costmap.grid
        grid_publisher.publish(msg)
     



if __name__=="__main__":
    #Topics
    grid_publisher = rospy.Publisher(Costmap.costmap_publish_topic, OccupancyGrid, queue_size=5)
    grid_update_publisher = rospy.Publisher(Costmap.costmap_update_publish_topic, OccupancyGrid, queue_size=5)
    #/Topics

    rospy.sleep(1)
    Costmap.publish()
    while not not rospy.is_shutdown():
        
        rospy.sleep(Costmap.seconds_per_update)
