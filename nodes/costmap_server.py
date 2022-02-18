#!/usr/bin/env python3

from types import new_class
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
    file = rospy.get_param('costmap_server/file','/cls.png')
    safe_footprint_radius =  rospy.get_param('costmap_server/safe_footprint_radius',0.30)
    ##
    #Topics
    grid_publisher = rospy.Publisher(cls.costmap_publish_topic, OccupancyGrid, queue_size=5)
    grid_update_publisher = rospy.Publisher(cls.costmap_update_publish_topic, OccupancyGridUpdate, queue_size=5)
    #/Topics
    costmap_publish_topic = rospy.get_param('costmap_server/costmap_publish_topic','/costmap')
    costmap_update_publish_topic = rospy.get_param('costmap_server/costmap_update_publish_topic','/costmap_update')
    #/Params

    color_image = cv2.imread("cls.png")
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
   
     
    #[cls.pixels[x+y] for x, y in zip(range(cls.height),range(cls.width))]
    # def __init__(self):
    #     cls.list.append(self)
    @classmethod
    def initCostmap(cls):
        start_time = rospy.Time.now()
        cls.grid = [[cls.pixels[x+y] for x in range(cls.height+1)] for y in range(cls.width+1)]
        for x,y in cls.grid:
            cls.inflate(x,y)
        cls.applied_list.clear()
        rospy.loginfo(f"Costmap init done in {(rospy.Time.now() - start_time).to_sec()}")
    @classmethod
    def publishUpdate(cls): ###An example
        msg = OccupancyGridUpdate()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.width = cls.width
        msg.height = cls.height
        data_list = []
        for x in range(cls.height):
            for y in range(cls.width):
                data_list.append(cls.grid[x][y])
        msg.data = data_list
        cls.grid_update_publisher.publish(msg)
    @classmethod
    def getInflationFromDist(cls,dist): ###dist in cells
        inflation = round(100*(cls.safe_footprint_radius/(dist**cls.inflation_power)))
        if inflation > 100:
            inflation = 100
        return  ####fix
    @classmethod
    def inflate(cls,x,y):
        cls.applied_list = []
        for d_x, d_y in cls.inflation_coords_list:
                next_x, next_y = x+d_x, y + d_y
                coord = (next_x,next_y)
                if not coord in cls.applied_list:
                    cls.applied_list.append(coord)
                    cls.grid[next_x][next_y] += cls.getInflationFromDist(np.linalg.norm(np.array([x,y]),np.array([next_x,next_y])))
    @classmethod
    def publish(cls): ###An example
        msg = OccupancyGrid()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution = cls.resolution
        msg.info.width = cls.width
        msg.info.height = cls.height
        msg.data = cls.grid
        cls.grid_publisher.publish(msg)



class Beacons:
    #Beacon params
    dist_threshhold = rospy.get_param('costmap_server/beacons/dist_threshhold',2) #in cells
    dist_between_double = rospy.get_param('costmap_server/beacons/dist_between_double',80) #in cells
    dist_from_double = rospy.get_param('costmap_server/beacons/dist_from_double', 155) #in cells (median of base of the triangle)
    #/Beacon params
    new_coords = []
    list = []
    pose = (0,0)
    def __init__(self,pos:tuple):
        Beacons.list.append(self)
    @classmethod
    def recieve(cls):
        Beacons.list.clear()
        for pose in cls.new_coords:
            beacon = Beacons(pose)

class Obstacle:
    #Obstacle params
    #/Obstacle params
    delta_coords = (0,0)
    def __init__(self,radius,resolution = 8):
        self.delta_coords = dCoordsInRad(radius,resolution)



def main():
    rate = rospy.Rate(Costmap.update_rate)
    rospy.sleep(1)
    Costmap.publish()
    while not not rospy.is_shutdown():
        
        rate.sleep()

if __name__=="__main__":
    main()
