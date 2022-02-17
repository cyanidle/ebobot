#!/usr/bin/env python3

import roslib
roslib.load_manifest('ebobot')
import rospy
from math import radians
import numpy as np
import tf

rospy.init_node('costmap_server')
##########
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
###########
from dorlib import dCoordsInRad
import numpy as np
import cv2


class Costmap():
    #Params
    inflation_power = rospy.get_param('costmap_server/inflation_power',2)
    update_rate = rospy.get_param('costmap_server/update_rate',0.5)
    inflation_radius = rospy.get_param('costmap_server/inflation_radius',0.3)
    inflation_step_radians_resolution = rospy.get_param('costmap_server/inflation_step_radians_resolution',0.2)
    resolution = rospy.get_param('costmap_server/resolution',0.05)
    file = rospy.get_param('costmap_server/file','/costmap.png')
    safe_footprint_radius =  rospy.get_param('costmap_server/safe_footprint_radius',0.30)
    ##
    #Topics
    grid_publisher = rospy.Publisher(Costmap.costmap_publish_topic, OccupancyGrid, queue_size=5)
    grid_update_publisher = rospy.Publisher(Costmap.costmap_update_publish_topic, OccupancyGridUpdate, queue_size=5)
    #/Topics
    costmap_publish_topic = rospy.get_param('costmap_server/costmap_publish_topic','/costmap')
    costmap_update_publish_topic = rospy.get_param('costmap_server/costmap_update_publish_topic','/costmap_update')
    #/Params

    color_image = cv2.imread("costmap.png")
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    pixels = np.around(np.divide(gray_image, 255.0), decimals=1)
    
    #Global
    width = 101
    height = 151
    applied_list = []
    inflation_coords_list = dCoordsInRad(inflation_radius//resolution,inflation_step_radians_resolution)
    grid = np.array([[0]*101 for _ in range(151)])
    #list = []
    #/Global
   
     
    #[Costmap.pixels[x+y] for x, y in zip(range(Costmap.height),range(Costmap.width))]
    # def __init__(self):
    #     Costmap.list.append(self)
    @staticmethod
    def initCostmap():
        Costmap.grid = [[Costmap.pixels[x+y] for x in range(Costmap.height+1)] for y in range(Costmap.width+1)]
        for x,y in Costmap.grid:
            Costmap.inflate(x,y)
        Costmap.applied_list.clear()
    @staticmethod
    def publishUpdate(): ###An example
        msg = OccupancyGridUpdate()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.width = Costmap.width
        msg.height = Costmap.height
        data_list = []
        for x in range(Costmap.height):
            for y in range(Costmap.width):
                data_list.append(Costmap.grid[x][y])
        msg.data = data_list
        Costmap.grid_update_publisher.publish(msg)
    @staticmethod
    def getInflationFromDist(dist): ###dist in cells
        return 100*((Costmap.safe_footprint_radius/(dist**Costmap.inflation_power))) ####fix
    @staticmethod
    def inflate(x,y):
        Costmap.applied_list = []
        for d_x, d_y in Costmap.inflation_coords_list:
                next_x, next_y = x+d_x, y + d_y
                coord = (next_x,next_y)
                if not coord in Costmap.applied_list:
                    Costmap.applied_list.append(coord)
                    Costmap.grid[next_x][next_y] += Costmap.getInflationFromDist(np.linalg.norm(np.array([x,y]),np.array([next_x,next_y])))
    @staticmethod
    def publish(): ###An example
        msg = OccupancyGrid()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution = Costmap.resolution
        msg.info.width = Costmap.width
        msg.info.height = Costmap.height
        msg.data = Costmap.grid
        Costmap.grid_publisher.publish(msg)

class Obstacle:
    delta_coords = (0,0)
    def __init__(self,radius,resolution = 8):
        self.delta_coords = dCoordsInRad(radius,resolution)

if __name__=="__main__":
    rate = rospy.Rate(Costmap.update_rate)
    

    rospy.sleep(1)
    Costmap.publish()
    while not not rospy.is_shutdown():
        
        rate.sleep()
