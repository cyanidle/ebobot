#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
import yaml


class Manager:
    rospy.init_node("task_manager")
    #Params
    file = rospy.get_param("~file", "/config/routes/route1.yaml")

    #/Params


    #Globals
    route = {}
    

    @classmethod
    def read(cls):
        with open(cls.file, "r") as stream:
            try:
                cls.route = (yaml.safe_load(stream))
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")