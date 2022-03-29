#!/usr/bin/env python3
#from nodes.laser_scan_callback import Objects
import roslib
roslib.load_manifest('ebobot')
import rospy
import numpy as np
import tf


##########
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
###########
from dorlib import dCoordsInRad
from ebobot.msg import Obstacles, Obstacle
#
import numpy as np
import cv2
import os
####
rospy.init_node('costmap_server')
_node_ready =0
#####

def obstaclesCallback(obst):
    if _node_ready:
        Objects.clear()
        for obj in obst.data:
            #if Costmap.debug:
                #print(f"Got new object {obj.y =} {obj.x =}")
            Objects((obj.y/Costmap.resolution, obj.x/Costmap.resolution),
            obj.radius/Costmap.resolution)
    Objects.updateMask() # AHAHAHAHAHAAHAHA H AHAHAHAHHAHAHAHAAHHAHAHHAHAHAAHAHAHAHAHAHAHAHAHAHAHAHHAHAHA Питон момент
        #короче здесь был лишний обступ и он обновлял и пересылал карту НА КАЖДОМ объекте, а.к.а. они появлялись по очереди и планер не всегда их видел вовремя))))
class Costmap():
    #Params
    #Features
    publish_on_obstacles_recieve = rospy.get_param('~publish_on_obstacles_recieve', 1)
    write_map_enable = rospy.get_param('~write_map_enable', 1)
    debug = rospy.get_param('~debug', 1)
    interpolate_enable = rospy.get_param('~interpolate_enable',1)
    inflate_enable = rospy.get_param('~inflate_enable', 0)
    if inflate_enable:
        rospy.logwarn("Please wait ~10 seconds, inflating...")
    super_debug = rospy.get_param('~super_debug',0)
    inflation_nonlinear_enable = rospy.get_param('~inflation_nonlinear_enable',0) 
    #/Features 
    inflation_threshhold = rospy.get_param('~inflation_threshhold',80) #from 0 to 100
    interpolation_radius = rospy.get_param('~interpolation_radius',2) #in cells
    base_inflation_coeff = rospy.get_param('~base_inflation_coeff',0.003) #VERY DANGEROUS
    
    inflation_nonlinear_power = rospy.get_param('~inflation_nonlinear_power',1)
    update_rate = rospy.get_param('~update_rate',2)
    inflation_radius = rospy.get_param('~inflation_radius',0.45)
    inflation_step_radians_resolution = rospy.get_param('~inflation_step_radians_resolution',5)
    resolution = rospy.get_param('~resolution',0.02)
    file = rospy.get_param('~file','costmap.png')
    file_dir = rospy.get_param('~file_dir','config/costmap/')
    #safe_footprint_radius =  rospy.get_param('~safe_footprint_radius',0.08)
    ##
    #Topics
    ######################################
    costmap_broadcaster = tf.TransformBroadcaster()
    costmap_publish_topic = rospy.get_param('~costmap_publish_topic','/costmap_server/costmap')
    costmap_update_publish_topic = rospy.get_param('~costmap_update_publish_topic','/costmap_server/updates')
    ######################################
    grid_publisher = rospy.Publisher(costmap_publish_topic, OccupancyGrid, queue_size=5)
    grid_update_publisher = rospy.Publisher(costmap_update_publish_topic, OccupancyGridUpdate, queue_size=5)
    #/Topics
    #/Params
    os.chdir(f"{file_dir}")
    color_image = cv2.imread(file)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)       
    pixels = np.around(np.divide(gray_image, 255.0/100), decimals=1)#np.rot90(np.around(np.divide(gray_image, 255.0/100), decimals=1))
    del gray_image
    # cv2.imwrite("map_read.png", pixels)
    # print(f"pixels = {pixels[0][0], pixels[0][1],pixels[1][0],pixels[1][1]}")
    # rospy.sleep(1)
    #Global
    inflation_radius_in_cells = inflation_radius/resolution
    #height = 151
    height = int(len(pixels))
    width = int(len(pixels[0]))
    grid = []
    mask = np.full((height,width),0)
    for _ in range(height):
        row = []
        for _ in range(width):
            row.append(0)
        grid.append(row)
    grid = np.array(grid)
    grid_parser = []
    for y in range(height):
        for x in range(width):
            #print (f"appending {y,x}")
            grid_parser.append((y,x))
    inflation_coords_list = dCoordsInRad(inflation_radius//resolution,inflation_step_radians_resolution)
    
    #/Global
   
    @classmethod
    def initCostmap(cls):
        start_time = rospy.Time.now()
        cls.grid = np.array([[0]*cls.width for _ in range(cls.height)])#cls.pixels#new_grid
        #if cls.debug:
            #rospy.loginfo(f"Map read \n(First row = {cls.grid[0]})\n(Row 40 = {cls.grid[39]})")
        if cls.inflate_enable:
            rospy.loginfo_once('Inflating...')
            for num, _tup in enumerate(cls.grid_parser):
                y,x = _tup
                #if y/50 >= 1.5 and x/50 >= 1.5:
                    #raise SyntaxError()
                cls.inflate(y,x)
                if not (num%100):
                    cls.publish() 
        else:
            cls.grid = cls.pixels
        if cls.interpolate_enable:
            cls.interpolateGrid()
        if cls.write_map_enable:
            #os.chdir(cls.file_dir)
            cv2.imwrite("inflated_costmap.png", cls.grid * 255/100)
            if cls.debug:
                mask = np.array([[0]*100 for _ in range(100)])
                for y,x in cls.inflation_coords_list:
                    mask[50+y][50+x] = 255
                cv2.imwrite("last_inflation_mask.png", mask)
            rospy.loginfo(f"Saving map into {cls.file_dir}\n map = {cls.grid}")
            
        rospy.logwarn(f"Costmap init done in {(rospy.Time.now() - start_time).to_sec()}")
    @classmethod
    def getInflation(cls,dist,y,x): ###dist in cells from origin x,y
        inflation = 0
        if dist:
            if cls.inflation_nonlinear_enable:
                inflation = cls.pixels[y][x] * cls.base_inflation_coeff * 1/(dist/cls.inflation_radius_in_cells)**cls.inflation_nonlinear_power
            else:
                inflation = cls.pixels[y][x] * cls.base_inflation_coeff * 1/(dist/cls.inflation_radius_in_cells)
        else:
            inflation = 100
        if inflation > 100:
            inflation = 100
        elif inflation < 0:
            inflation = 0
        return inflation
    @classmethod
    def interpolateGrid(cls):
        new_grid = np.array([[0]*cls.width for _ in range(cls.height)])
        rospy.loginfo_once('Interpolating...')
        interpolation_list = []
        for i in range(-cls.interpolation_radius,cls.interpolation_radius+1):
            for j in range(-cls.interpolation_radius,cls.interpolation_radius+1):
                if not (i==0 and j==0):
                    interpolation_list.append((i,j))
        new_grid = cls.grid
        for y,x in cls.grid_parser:
            num = 0 
            sum = 0
            for dy, dx in interpolation_list:
                rospy.loginfo_once('Interpolation working...')
                new_y,new_x = y+dy,x+dx
                if (cls.interpolation_radius < new_x < cls.width-cls.interpolation_radius-1 and new_x 
                 and cls.interpolation_radius < new_y  < cls.height-cls.interpolation_radius-1 and new_y):
                    sum += cls.grid[new_y][new_x]
                    num += 1
            if num:
                new_grid[y][x] = int(round(sum/num))
                if new_grid[y][x] > 100:
                   new_grid[y][x] = 100
        cls.grid = new_grid
        #new_grid.clear()
    @classmethod
    def inflate(cls,y,x):
        #new_grid = np.array([[0]*cls.width for _ in range(cls.height)])
        if cls.pixels[y][x] > cls.inflation_threshhold and not rospy.is_shutdown():
            if  cls.width -1 > x > 1 and  1 < y < cls.height - 1: #check if surrounded
                sum = 0
                for dy in range(-1,2):
                    for dx in range(-1,2):
                        sum += cls.pixels[y+dy][x+dx] > cls.inflation_threshhold
                if sum > 8:
                    return  
            for d_y, d_x in cls.inflation_coords_list:
                next_y, next_x = y+d_y, x + d_x
                if (next_x >= 0 and next_x < cls.width) and (next_y>=0 and next_y <cls.height):    
                    dist = np.linalg.norm(np.array([next_y,next_x]-np.array([y,x])))
                    inflation = cls.getInflation(dist,y,x)
                    cls.grid[next_y][next_x] += inflation
                    if cls.grid[next_y][next_x] > 100:
                        cls.grid[next_y][next_x] = 100
    @classmethod
    def publishUpdate(cls): ###An example
        msg = OccupancyGridUpdate()
        msg.header.frame_id = "costmap" ###????????
        msg.header.stamp = rospy.Time.now()
        msg.height = cls.height
        msg.width = cls.width
        data_list = []
        for x in range(cls.width):
            for y in range(cls.height):
                data_list.append(cls.grid[y][x])
        msg.data = data_list
        cls.grid_update_publisher.publish(msg)

               
    @classmethod
    def publish(cls): ###An example
        #rospy.loginfo(f"Publishing map")
        msg = OccupancyGrid()
        curr_time = rospy.Time.now()
        msg.header.frame_id = "costmap" ###????????
        msg.header.stamp = curr_time 
        msg.info.resolution = cls.resolution
        msg.info.height = cls.height
        msg.info.width = cls.width
        for y, x in cls.grid_parser:
            #print(x,y)
            data = int(cls.grid[y][x] + cls.mask[y][x])
            if data > 100:
                data = 100
            msg.data.append(data)
        
        cls.costmap_broadcaster.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0,0,0),
            curr_time ,
            "costmap",
            "map" ##???
            )
        cls.grid_publisher.publish(msg)


class Objects:
    #Obstacle params
    #Features
    use_default = rospy.get_param('~obstacles/use_default',0)
    resolution = rospy.get_param('~obstacles/resolution',15)
    #
    base_inflation_coeff = rospy.get_param('~obstacles/base_inflation_coeff',150)
    #
    topic = rospy.get_param('laser_scan_callback/obstacles/list_topic','/laser/obstacles')
    ######################################
    subscriber = rospy.Subscriber(topic, Obstacles, obstaclesCallback)
    #/Obstacle params

    #globals
    list = []

    def __init__(self,pos,radius, default = 0):
        if not Objects.use_default:
            self.delta_coords = dCoordsInRad(radius,Objects.resolution)
            self.pos = pos
            self.inflation = []
            self.inflation.append(100)
            for y,x in self.delta_coords[1:]:
                infl = Objects.base_inflation_coeff/(np.linalg.norm((y,x)))
                if infl > 100:
                    infl = 100
                self.inflation.append(infl)
            if not default:
                Objects.list.append(self)
        elif default:
            self.delta_coords = dCoordsInRad(radius,Objects.resolution)
            self.pos = pos
            self.inflation = [100]
            for y,x in self.delta_coords[1:]:
                infl = Objects.base_inflation_coeff/(np.linalg.norm((y,x)))
                if infl > 100:
                    infl = 100
                self.inflation.append(infl)
        else:
            self.pos = pos
            Objects.list.append(self)

    @staticmethod
    def clear():
        Objects.list.clear()
    @staticmethod
    def updateMask():
        #rospy.loginfo(f"Updating mask")
        Costmap.mask = np.full((Costmap.height,Costmap.width),0)
        #print (Costmap.mask)
        if not Objects.use_default:
            for obst in Objects.list:
                 for coords,inflation  in zip(obst.getCoords(),obst.inflation):
                    y, x  = coords
                    y, x = int(y), int(x)
                    if 0 < x < Costmap.width and 0 < y < Costmap.height:
                        Costmap.mask[y][x] += inflation
                        if Costmap.mask [y][x] > 100:
                            Costmap.mask [y][x] = 100
        else:
            for obst in Objects.list:
                default_obstacle.pos = obst.pos
                for coords,inflation in zip(default_obstacle.getCoords(),default_obstacle.inflation):
                    y, x  = coords
                    y,x = int(y), int(x)
                    if 0 < x < Costmap.width and 0 < y < Costmap.height:
                        Costmap.mask[y][x] += inflation
                        if Costmap.mask [y][x] > 100:
                            Costmap.mask [y][x] = 100
        if Costmap.publish_on_obstacles_recieve:
            Costmap.publish()
    def getCoords(self) -> list:
        for y,x in self.delta_coords:
            yield  (self.pos[0] + y , self.pos[1] + x)       


def main():
    if Costmap.debug:
        rospy.loginfo(f"Costmap shape is {Costmap.height,Costmap.width}(w,h)")
        rospy.loginfo(f"Costmap is\n{Costmap.pixels}")
        rospy.loginfo(f"Inflation radius in cells = {Costmap.inflation_radius_in_cells}")
    rate = rospy.Rate(Costmap.update_rate)
    Costmap.publish()
    while not rospy.is_shutdown():
        if not Costmap.publish_on_obstacles_recieve:
            Costmap.publish()
        rate.sleep()

if __name__=="__main__":
    Costmap.initCostmap()
    _node_ready = 1
    if Objects.use_default:
        rospy.loginfo(f"Using default obstacles, initialazing...")
        start_time = rospy.Time.now()
        default_obstacle = Objects((-10,-10), 10, default=1)
        rospy.loginfo(f"Done in {(rospy.Time.now()-start_time).to_sec()}!")
    main()
