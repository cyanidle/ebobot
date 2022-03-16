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
from calls_executer import Move as move_client_constructor
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
            def __init__(self,name,args:tuple):
                self.args = args
                self.call, self.status_name = executer_dict()[name]
                self.curr_status = None
                Status.add(self)
                return self.exec 
            def exec(self):
                self.curr_status = self.call(self.args)
            def status(self):
                return self.curr_status
            def statusUpdate(self):
                pass
        class Move:
            client = move_client_constructor()
            def __init__(self,pos):
                self.pos = pos
            def exec(self):
                self.curr_status = self.call(self.args)
            def status(self):
                return self.curr_status
            def statusUpdate(self):
                pass
        class Logs:
            def __init__(self,args:str):
                #_, val = micro
                self.text = args
                return self.exec
            def exec(self,args):
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
                    self.curr_status = "bad"
                self.curr_status = "good"
            def status(self):
                return self.curr_status
            def statusUpdate(self):
                pass
        class Skip:
            def __init__(self,pos):
                self.pos = pos
            def exec(self):
                self.curr_status = self.call(self.args)
            def status(self):
                return self.curr_status
            def statusUpdate(self):
                pass
        class Together:
            def __init__(self,pos):
                self.pos = pos
            def exec(self):
                self.curr_status = self.call(self.args)
            def status(self):
                return self.curr_status
            def statusUpdate(self):
                pass
        ############ Microtask
        def __init__(self,key,args):
            rospy.loginfo(f"Initialising microtask {key = } {args = }")
            self.action = Manager.constructors_dict[key](args)
            return self
        @staticmethod
        def getExec(key,args): #val in the dict!
            micro = Task.Microtasks(key,args)#each constructor 
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
    list = []
    def __init__(self):
        super().__init__(self)
        Interrupts.list.append(self)
    def forceCallParse(self):
        return self.action
    def forceCall(self):
        for micro in self.micros_list:
            micro.action()
    pass
    def parseCond(self,arg):
        pass
    @classmethod
    def update(cls):
        pass
########################
class Status:    ### EACH OBJECT WHICH IS ADDED TO STATUS SERVER SHOULD HAVE A STATUS AND UPDATE STATUS METHOD, AND A STATUS_NAME ATTRIBUTE
    dict = []          ### STATUS RETURNS THE DESIRED VALUE, WHILE UPDATE JUST GETS CALLED EACH STATUS SERVER UPDATE CYCLE
    class Timer:     
        def __init__(self) -> None:
            self.time = 0
            self.ros_time = rospy.Time.now()
            Status.addStatus(self)
        def status(self):
            return self.time
        def updateStatus(self):
            self.time = (rospy.Time.now() - self.ros_time).to_sec()
    @staticmethod
    def add(obj):
        Status.list.append(obj)
    @staticmethod
    def update():
        for obj in Status.list:
            obj.updateStatus()


    
######################
class Manager:

    #Params
    update_rate = rospy.get_param("~update_rate", 5)
    file = rospy.get_param("~file", "/config/routes/route1.yaml")
    #/Params
    #Globals
    route = {}
    constructors_dict = {
        "call": Task.Microtasks.Calls,
        "log": Task.Microtasks.Logs,
        "move": Task.Move,
        "condition":Task.Microtasks.Conditions,
        "together": Task.Microtasks.Together,
        "interrupt": Interrupts.forceCallParse,
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
    Manager.parse()
    timer = Status.Timer()
    rate = rospy.Rate(Manager.update_rate)
    while not rospy.is_shutdown():
        Status.update()
        rate.sleep()



####
if __name__=="__main__":
    main()
