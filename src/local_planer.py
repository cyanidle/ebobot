from types import prepare_class
import roslib
roslib.load_manifest('ebobot')
import rospy
import cmath
import tf
rospy.init_node('local_planer')
from libraries.Dorvect import Dorvect
#The planer should try planing the path using radius of the robot and avoiding obstacles, but if the next point is unreachable, skip it and reroute
#To the next

class Local():
    #Params
    debug = rospy.get_param('local_planer/debug', 1)
    path_subscriber_topic =  rospy.get_param('local_planer/path_publisher_topic', 'global_path') #### TAKEN FROM GLOBAL_PLANER PARAMS
    num_of_circles = rospy.get_param('local_planer/path_publisher_topic', 3)
    step_radians = rospy.get_param('local_planer/step_radians', cmath.pi/4)
    #### Params for footprint cost calc
    footprint_calc_step_radians = rospy.get_param('local_planer/base_footprint_radius', 0.3)
    #calculate_base_cost = rospy.get_param('local_planer/calculate_base_cost', 0.3)
    base_footprint_radius = rospy.get_param('local_planer/base_footprint_radius', 0.20) #optional
    safe_footprint_radius = rospy.get_param('local_planer/safe_footprint_radius', 0.25)
    #### /Params for footprint cost calc
    circles_dist = rospy.get_param('local_planer/circles_dist', base_footprint_radius/num_of_circles)
    #/Params

    #Global values
    x_list =[]
    cost_coords_list = []
    #/Global values

    #def __init__(self):
    def recalcCostCoordsFromRadius(radius):
        Local.x_list.clear()
        Local.cost_coords_list.clear()
        Local.precalcCostCoordsFromRadius(radius)
    def precalcCostCoordsFromRadius(radius):
        rad = radius                           #Local.safe_footprint_radius
        vect = Dorvect(rad,0,0)
        x_list = []
        x_list.append((0,rad))
        last_x = 0
        for i in range(cmath.pi//(Local.step_radians*2)+1): #This shit here casts some vectors with dist = rad in a quarter circle #Should fix for different rad steps.  
            vect.updateFromImag(vect.imag * (cmath.cos(-Local.step_radians) + 1j * cmath.sin(-Local.step_radians))) #This may cause a bug later (excess coords) or (missing coords) near y=0, x = rad
            for x in range(last_x,round(vect.x)+1):
                if Local.debug:
                    rospy.loginfo(f"Precalc max x --> {x}, max y --> {round(vect.y)}")
                x_list.append((x, round(vect.y)))
            last_x = round(vect.x)
        for x_max, y_max in Local.x_list:
            for x in range(-x_max,x_max+1):
                for y in range(-y_max,y_max+1):
                    Local.cost_coords_list.append((x,y))
    def getCost():
    def getPoses():
    def cmdVel():

if __name__ == "__main__":
    #epic