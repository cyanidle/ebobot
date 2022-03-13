#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
import yaml
import asyncio
rospy.init_node("task_manager")
#
from markers import pubMarker
from calls_executer import executer_dict
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
#


    
########## Subclasses
class Task:
    list =  []
    @classmethod
    def parseMicroList(cls,list:list) -> list:
        micro_list = []
        for dict in list: ####
            entry_name = list(dict)[0]
            entry_value = dict[entry_name]
            micro_list.append(   (entry_name, entry_value)   )
        return micro_list
    class Microtasks:                #EACH ENTRY'S CONSTRUCTOR MUST RETURN A CALLABLE OBJECT, 
        class Conditions:            #WHICH EXECUTES THE COMMAND AND ONLY THEN RETURNS TO MAIN
            if_list = []             #EXECUTING THE OBJECT SHALL RETURN STATUS (BAD/GOOD/CUSTOM)
            else_list = []
            def __init__(self,arg):
                items = arg.items()
                self.check = self.parseIf(items[0])
                self.yes = self.parseDo(items[1])
                self.no = self.parseElse(items[2])
            def parseIf(self,args):



                pass                    # PLS DO LATER
            def parseDo(self,args):
                return Task.Microtasks.getExec(args)
            def parseElse(self,args):
                return Task.Microtasks.getExec(args)
            def exec(self):
                if self.check():
                    return self.yes
                else:
                    return self.no
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
            dict = executer_dict()
            def __init__(self,name,args:tuple):
                self.args = args
                self.call = type(self).dict[name]
                return self.exec 
            def exec(self):
                self.call(self.args)
        class Moves:
            def __init__(self,pos):
                self.pos = pos
            pass
        class Logs:
            def __init__(self,val:str):
                #_, val = micro
                self.text = val
                return self.exec
            def exec(self):
                text = self.text
                pref = text[:2]
                if pref == "L:":
                    rospy.loginfo(text[2:])
                elif pref == "W:":
                    rospy.logwarn(text[2:])
                elif pref == "E:":
                    rospy.logerr(text[2:])
                else:
                    rospy.logerr(f"(Incorrect log prefix!) {text}")
                    return "bad"
                return "good"

        class Skip:
            pass
        class Together:
            pass
        ############ Microtask
        def __init__(self,key,arg):
            self.action = Manager.constructors_dict[key](arg)
            return self
        @staticmethod
        def getExec(key,arg): #val in the dict!
            micro = Task.Microtasks(key,arg)#each constructor 
            return micro                     #should return an executable object
    ######### Task
    def __init__(self,name:str, list:list):
        "Inits a new task from dict, parses microtasks and appends them into own (micros_list)"
        self.name = name
        for name, val in self.parseMicroList(name,val):
            self.micro_list.append(Task.Microtasks.getExec(val)) #micro is a tuple (key, val) for current dict position
        Task.list.append(self)

    #pass
######################
class Interrupts(Task):
    def __init__(self):
        super().__init__(self)
        InterruptsServer.list.append(self)
    def forceParse(self):
        return self.action
    def forceCall(self):
        for micro in self.micros_list:
            micro.action()
    pass
class InterruptsServer:
    list = []
    pass
class StatusServer:
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
        "together": Task.Microtasks.Together,
        "interrupt": Interrupts.forceParse,
        "skip": Task.Microtasks.Skip
        }
    ##################### Manager
    # def __init__(self):
    #     sesdasd
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
    @staticmethod
    def exec(action):
        action()




def main():
    Manager.read()

####
if __name__=="__main__":
    main()
