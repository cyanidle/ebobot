#!/usr/bin/env python3
import rospy
import importlib
import roslib
roslib.load_manifest('ebobot')

from global_planer import Global
from costmap_server import Costmap

if __name__=="__main__":
    Global.appendNextPos()
    rospy.loginfo(f"Global: {dir(Global)}") 
    rospy.loginfo(f"Costmap: {dir(Costmap)}") 