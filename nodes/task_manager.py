#!/usr/bin/env python3
#from multiprocessing import Condition
import roslib
roslib.load_manifest('ebobot')
import rospy
import yaml
#from dataclasses import dataclass
rospy.init_node("task_manager")
#
from markers import pubMarker
#

class Condition:
        def __init__(self,empty = False):

            pass
        pass
class Manager:
    
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

    @classmethod
    def parse(cls):
        pass

    

     
    class Task:
        def __init__(self,num:int,actions:list, log:str = "None",condition:Condition = Condition(empty = True)):
            pass

    #@dataclass(frozen=True)
    class Interrupt(Task):
        pass

if __name__=="__main__":
    Manager.read()