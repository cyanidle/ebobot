#!/usr/bin/env python3
#from multiprocessing import Condition
import roslib
roslib.load_manifest('ebobot')
import rospy
import yaml
import asyncio
#from dataclasses import dataclass
rospy.init_node("task_manager")
#
from markers import pubMarker
from calls_dict import calls_dict
#



    
########## Subclasses
class Task:
    micros_list = []
    @classmethod
    def parseMicroList(cls,list:list) -> list:
        micro_list = []
        for dict in list: ####
            entry_name = list(dict)[0]
            entry_value = dict[entry_name]
            micro_list.append(   (entry_name, entry_value)   )
        return micro_list
    class Microtasks:     #EACH ENTRY'S CONSTRUCTOR MUST RETURN A CALLABLE OBJECT, 
        class Conditions: #WHICH EXECUTES THE COMMAND AND ONLY THEN RETURNS TO MAIN
            class Do:
                if_list = []
                else_list = []
                def __init__(self, list:list) -> None:
                    self.list = Task.parseMicroList(list)
                    pass
                def parseIf(self):
                    pass
                def parseElse(self):
                    pass
            ####Conditions
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
        class Skip:
            pass
        class Together:
            pass
        ############ Microtask
        def __init__(self,micro:tuple):
            name, val = micro
            self.name = name
            self.action = Manager.constructors_dict[name](val) #each constructor 
            return self          #should return an executable object
    ######### Task
    def __init__(self,name:str, list:list):
        "Inits a new task from dict, parses microtasks and appends them into own (micros_list)"
        self.name = name
        for micro in self.parseMicroList(list):
            self.micro_list.append(Task.Microtasks(micro))
    

    #pass
######################
class Interrupts(Task):
    def forceParse(self):
        return self.action
    def forceCall(self):
        for micro in self.micros_list:
            micro.action()
    pass
######################
class Manager:
    #Params
    file = rospy.get_param("~file", "/config/routes/route1.yaml")
    #/Params
    #Globals
    route = {}
    constructors_dict = {
        "call": Task.Microtasks.Calls,
        "log": Task.Microtasks.Logs,
        "condition":Task.Microtasks.Conditions,
        "do": Task.Microtasks.Conditions.Do.parseIf, 
        "else": Task.Microtasks.Conditions.Do.parseElse,
        "together": Task.Microtasks.Together,
        "interrupt": Interrupts.forceParse,
        "skip": Task.Microtasks.Skip
        }
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
            unparsed_list = cls.route["tasks"][task_name]
            new_task = Task(unparsed_list,task_name)


if __name__=="__main__":
    Manager.read()