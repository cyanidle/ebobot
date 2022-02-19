#!/usr/bin/env python3
from turtle import width
import roslib
roslib.load_manifest('ebobot')
import rospy
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
    debug = rospy.get_param('costmap_server/debug',1)
    inflation_buff_coeff = rospy.get_param('costmap_server/inflation_buff_coeff',0.2)
    inflation_power = rospy.get_param('costmap_server/inflation_power',2)
    update_rate = rospy.get_param('costmap_server/update_rate',0.5)
    inflation_radius = rospy.get_param('costmap_server/inflation_radius',0.3)
    inflation_step_radians_resolution = rospy.get_param('costmap_server/inflation_step_radians_resolution',0.2)
    resolution = rospy.get_param('costmap_server/resolution',0.02)
    file = rospy.get_param('costmap_server/file','/home/alexej/catkin_ws/src/ebobot/nodes/costmap.png')
    safe_footprint_radius =  rospy.get_param('costmap_server/safe_footprint_radius',0.30)
    ##
    #Topics
    costmap_broadcaster = tf.TransformBroadcaster()
    costmap_publish_topic = rospy.get_param('costmap_server/costmap_publish_topic','/costmap')
    costmap_update_publish_topic = rospy.get_param('costmap_server/costmap_update_publish_topic','/costmap_update')
    grid_publisher = rospy.Publisher(costmap_publish_topic, OccupancyGrid, queue_size=5)
    grid_update_publisher = rospy.Publisher(costmap_update_publish_topic, OccupancyGridUpdate, queue_size=5)
    #/Topics
    #/Params

    color_image = cv2.imread(file)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)       
    pixels = np.around(np.divide(gray_image, 255.0/100), decimals=1)
    
    
    #Global
    width,height = int(len(pixels[0])), int(len(pixels))
    grid_parser = []
    for x in range(height):
        for y in range(width):
            grid_parser.append((x,y))
    if debug:
        rospy.loginfo(f"Costmap shape is {width,height}(w,h)")
    #applied_list = []
    inflation_coords_list = dCoordsInRad(inflation_radius//resolution,inflation_step_radians_resolution)
    
    grid = np.array([[0]*101 for _ in range(151)])
    #list = []
    #/Global
   
    @classmethod
    def initCostmap(cls):
        start_time = rospy.Time.now()
        cls.grid = np.array(cls.pixels)#new_grid
        if cls.debug:
            rospy.loginfo(f"Map read (First row = {cls.grid[0]})")
        for x,y in cls.grid_parser:
            if cls.debug:
                rospy.loginfo(f"Inflating {x},{y}")
            cls.inflate(x,y)
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
        if dist:
            inflation = round(100 - cls.inflation_buff_coeff*dist**cls.inflation_power)
        else:
            inflation = 100
        # if inflation > 100:
        #     inflation = 100
        return  inflation
    @classmethod
    def inflate(cls,x,y):
        for d_x, d_y in cls.inflation_coords_list:
                next_x, next_y = abs(x+d_x), abs(y + d_y)
                if (next_x > 0 and next_x < cls.height) and (next_y>0 and next_y <cls.width):
                    inflation = cls.getInflationFromDist(np.linalg.norm(np.array([next_x,next_y]-np.array([x,y]))))
                    if cls.debug:
                        rospy.loginfo(f"x {next_x}, y {next_y}, inflation = {inflation}")
                    cls.grid[next_x][next_y] += inflation
    @classmethod
    def publish(cls): ###An example
        msg = OccupancyGrid()
        curr_time = rospy.Time.now()
        msg.header.frame_id = "/map" ###????????
        msg.header.stamp = curr_time 
        msg.info.resolution = cls.resolution
        msg.info.width = cls.width
        msg.info.height = cls.height
        for x in range(cls.height):
            for y in range(cls.width):
                msg.data.append(cls.grid[x,y])
        cls.costmap_broadcaster.sendTransform(
            (0, 0, 0.),
            (0,0,0,0),
            curr_time ,
            "fixed",
            "map"
            )
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
        self.pose = pos
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
    Costmap.initCostmap()
    Costmap.publish()
    
    while not rospy.is_shutdown():
        Costmap.publish()
        rate.sleep()

if __name__=="__main__":
    main()
