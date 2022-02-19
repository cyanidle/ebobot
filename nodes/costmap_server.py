#!/usr/bin/env python3
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
import os

class Costmap():
    #Params
    interpolation_radius = rospy.get_param('costmap_server/enable_interpolation',1) #in cells
    interpolate_enable = rospy.get_param('costmap_server/interpolate_enable',1)
    base_inflation_coeff = rospy.get_param('costmap_server/base_inflation_coeff',0.015)
    super_debug = rospy.get_param('costmap_server/super_debug',0)
    inflate_enable = rospy.get_param('costmap_server/inflate_enable',1)
    debug = rospy.get_param('costmap_server/debug',1)
    write_map_enable = rospy.get_param('costmap_server/write_map_enable', 1 )
    inflation_nonlinear_enable = rospy.get_param('costmap_server/inflation_nonlinear_enable',0) 
    inflation_nonlinear_power = rospy.get_param('costmap_server/inflation_nonlinear_power',1)
    update_rate = rospy.get_param('costmap_server/update_rate',0.5)
    inflation_radius = rospy.get_param('costmap_server/inflation_radius',0.4)
    inflation_step_radians_resolution = rospy.get_param('costmap_server/inflation_step_radians_resolution',5)
    resolution = rospy.get_param('costmap_server/resolution',0.02)
    file = rospy.get_param('costmap_server/file','/home/alexej/catkin_ws/src/ebobot/nodes/costmap/costmap.png')
    file_dir = rospy.get_param('costmap_server/file_dir','/home/alexej/catkin_ws/src/ebobot/nodes/costmap/')
    #safe_footprint_radius =  rospy.get_param('costmap_server/safe_footprint_radius',0.08)
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
    inflation_radius_in_cells = inflation_radius/resolution
    width,height = int(len(pixels[0])), int(len(pixels))
    grid_parser = []
    for x in range(height):
        for y in range(width):
            grid_parser.append((x,y))
    
    inflation_coords_list = dCoordsInRad(inflation_radius//resolution,inflation_step_radians_resolution)
    if debug:
        rospy.loginfo(f"Costmap shape is {width,height}(w,h)")
        rospy.loginfo(f"Costmap is {pixels}")
        rospy.loginfo(f"Inflation radius in cells = {inflation_radius_in_cells}")
    grid = np.array([[0]*101 for _ in range(151)])
    #/Global
   
    @classmethod
    def initCostmap(cls):
        start_time = rospy.Time.now()
        cls.grid = cls.pixels#new_grid
        #if cls.debug:
            #rospy.loginfo(f"Map read \n(First row = {cls.grid[0]})\n(Row 40 = {cls.grid[39]})")
        if cls.inflate_enable:
            for x,y in cls.grid_parser:
                cls.inflate(x,y)
            if cls.write_map_enable:
                os.chdir(cls.file_dir)
                cv2.imwrite("inflated_costmap.png", cls.grid * 255/100)
                if cls.debug:
                    mask = np.array([[0]*100 for _ in range(100)])
                    for x, y in cls.inflation_coords_list:
                        mask[50+x][50+y] = 255
                    cv2.imwrite("last_inflation_mask.png", mask)
                rospy.loginfo(f"Saving map into {cls.file_dir}\n map = {cls.grid}")
            if cls.interpolate_enable:
                cls.interpolateGrid()
        rospy.loginfo(f"Costmap init done in {(rospy.Time.now() - start_time).to_sec()}")
    @classmethod
    def getInflation(cls,dist,x,y): ###dist in cells from origin x,y
        inflation = 0
        if dist:
            if cls.inflation_nonlinear_enable:
                inflation = cls.pixels[x][y] * cls.base_inflation_coeff * (dist/cls.inflation_radius_in_cells)**cls.inflation_nonlinear_power
            else:
                inflation = cls.pixels[x][y] * cls.base_inflation_coeff * (dist/cls.inflation_radius_in_cells)
        #else:
            #inflation = 100
        if inflation > 100:
            inflation = 100
        elif inflation < 0:
            inflation = 0
        return inflation
    @classmethod
    def interpolateGrid(cls):
        interpolation_list = []
        for i in range(cls.interpolation_radius):
            for j in range(cls.interpolation_radius):
                if not (i==0 and j==0):
                    interpolation_list.append((i,j))
        new_grid = cls.grid
        for x,y in cls.grid_parser:
            num = 0
            sum = 0
            for dx, dy in interpolation_list:
                new_x,new_y = x+dx,y+dy
                if new_x < cls.height and new_x >= 0 and new_y  < cls.width and new_y  >= 0:
                    sum += cls.grid[new_x][new_y]
                    num += 1
            new_grid[x][y] = sum/num
        cls.grid = new_grid
    @classmethod
    def inflate(cls,x,y):
        if cls.pixels[x][y] > 50:
            if x > 1 and y > 1 and x < cls.height -1 and y < cls.width - 1: #check if surrounded
                sum = 0
                for dx in range(-1,2):
                    for dy in range(-1,2):
                        sum += cls.pixels[x+dx][y+dy]
                if sum > 99*9:
                    return  
            for d_x, d_y in cls.inflation_coords_list:
                next_x, next_y = x+d_x, y + d_y
                if (next_x > 0 and next_x < cls.height) and (next_y>0 and next_y <cls.width):    
                    dist = np.linalg.norm(np.array([next_x,next_y]-np.array([x,y])))
                    inflation = cls.getInflation(dist,x,y)
                    if cls.super_debug:
                        rospy.loginfo(f"x {next_x}, y {next_y}, inflation = {inflation}, dist = {dist}")
                    cls.grid[next_x][next_y] += inflation
                    if cls.grid[next_x][next_y] > 100:
                        cls.grid[next_x][next_y] = 100
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
                msg.data.append(int(cls.grid[x,y]))
        zero_quat = tf.transformations.quaternion_from_euler(0,0,0)
        cls.costmap_broadcaster.sendTransform(
            (0, 0, 0),
            zero_quat,
            curr_time ,
            "costmap",
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
