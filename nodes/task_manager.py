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


class Manager:
    
    class Task:
        def __init__(self, conditions, call_list, move_list, log_list):
            pass
        class Calls:
            pass
        class Moves:
            pass
        class Logs:
            pass
        class Conditions:
            def __init__(self, move_index, call_index):
                self.move_index = move_index
                self.call_index = call_index
                pass
            pass
            def __init__(self,num:int,actions:list, log:str = "None",condition:Condition = Condition(empty = True)):
                pass

    #@dataclass(frozen=True)
    class Interrupts(Task):
        def __init__():

            pass
        pass
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
        #interrupts = cls.route["interrupts"]
        #tasks = cls.route["tasks"]
        for interrupt in cls.route["interrupts"]:
            pass
        for task in cls.route["tasks"]:
            pass

    

    

if __name__=="__main__":
    Manager.read()