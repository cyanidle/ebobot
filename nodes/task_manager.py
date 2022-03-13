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
from calls_dict import calls_dict
#


class Manager:
    #Params
    file = rospy.get_param("~file", "/config/routes/route1.yaml")
    #/Params
    #Globals
    route = {}
    ##################### Manager
    def __init__():
        pass
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
        for interrupt_name in cls.route["interrupts"]:
            pass
        for task_name in cls.route["tasks"]:
            new_task = Task(cls.route["tasks"][task_name],task_name)

    
########## Subclasses
class Task:
    micro_list = []
    class Microtasks:
        class Conditions:
            def __init__(self, move_index, call_index, empty = True):
                if empty:
                    pass
                else:
                    self.move_index = move_index
                    self.call_index = call_index
                pass
            pass
        class Calls:
            dict = {}
            def __init__(self,name,args:tuple):
                self.name = str(name)
                self.exec = type(self).dict[name]
                self.args = args
                pass
            @classmethod
            def parse(cls):
                cls.dict = calls_dict #FINISH THIS
            pass
        class Moves:
            def __init__(self,pos):
                self.pos = pos
            pass
        class Logs:
            def __init__(self,text):
                pass
            pass
        ############ Microtask
        def __init__(self):
            pass
    ######### Task
    def __init__(self,name:str, list:list):
        self.name = name
        for micro in self.parseMicroList(list):
            self.micro_list.append(micro)
    @classmethod
    def parseMicroList(cls,list:list):
        pass
    #pass
######################
class Interrupts(Task):
    pass


if __name__=="__main__":
    Manager.read()