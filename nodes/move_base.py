#!/usr/bin/env python3
import rospy
import importlib
import roslib
roslib.load_manifest('ebobot')

from global_planer import Global


if __name__=="__main__":
    rospy.loginfo(f"Costmap: {dir(Global)}") 