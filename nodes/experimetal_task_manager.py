#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
import yaml
from threading import Thread
import asyncio
rospy.init_node("task_manager")
#
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponce
#
from markers import pubMarker
from calls_executer import executer_dict, showPrediction
from calls_executer import Move as move_client_constructor
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
#
from abc import ABC, abstractmethod


class Status:
    list = []
    deps_dict = {}
    def __init__(self, parent):
        self._status = "init"
        self.parent = parent
    def update(self):
        self._status = self.parent.updateStatus()
    def get(self):
        return self._status
    def set(self, text):
        self._status = text
    @staticmethod
    def checkDeps(obj):
        obj_str = Status.getRawString(obj)
        if Status.deps_dict[obj_str]:
            for dep in Status.deps_dict[obj_str]:
                dep.trigger()
    @staticmethod
    def getRawString(obj):
        return str(obj)
        #return str(obj.parent.name) + "/" + str(obj.name) + "/"+ str(obj.num)
    @staticmethod
    def registerDependency(obj, condition:str):
        rospy.loginfo(f"Adding dependecy of {obj} from condition {condition}")
        if Status.deps_dict[condition]:
            Status.deps_dict[condition].append(obj) 
        else:
            Status.deps_dict[condition] = [obj]
    @staticmethod
    def clearDependency(obj, condition:str):
        if Status.deps_dict[condition]:
            Status.deps_dict[condition].remove(obj) 
        else:
            Status.deps_dict[condition] = []
#######################################################
class Template(ABC):
    list = []
    class_name = "<NameNotSet>"
    @abstractmethod
    def __init__(self,parent,name,args):
        rospy.loginfo(f"Initialising {name} with args {args}, inside of {parent}")
        self.parent = parent
        self._activate_flag = False
        self.num = 0
        if type(parent) == Task:
            for self.name in self.parent.micros_list:
                self.num += 1
        else:
            self.num = len(type(self).list)
        type(self).list.append(self)
        self.name = name
        self.status = Status(self)
    ############################ Should not be overriden
    async def exec(self)->None:
        self.status.set("executing")
        ############### Done before ^^^
        self.midExec()
        ############### Done after main exec
        self._activate_flag = False
        Status.checkDeps(self)
    ############################
    @abstractmethod
    async def midExec(self)-> None:
        raise NotImplementedError(self.midExec)
    ####### Not neccessary to override
    def trigger(self) -> None:
        self._activate_flag = True
    def updateStatus(self):
        return self.status.get()  
    def __str__(self) -> str:
        return f"{self.parent.name}/{self.name}/{self.num}"
    def __repr__(self):
        return f"<{type(self).class_name} {self.num}: {self.name}>"
########################################################
class Call(Template):
    class_name = "Call"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
    async def midExec(self) -> None:
        pass
class DynamicCall(Call):
    class_name = "Dynamic call"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
    async def midExec(self) -> None:
        pass
########################################################
class Condition(Template):
    class_name = "Condition"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        check_args = args[0]["if"]
        self.yes = self.parse(args[1]["do"])
        self.no = self.parse(args[2]["else"])
        Status.registerDependency(self,check_args)
    def parse(self,args):
        sub_name, sub_args = Task.parseMicroList(args)
        return Task(self,sub_name,sub_args)
    def trigger(self):
        self._activate_flag= True
    async def midExec(self) -> None:
        if self._activate_flag:
            proc = asyncio.create_task(self.yes.exec())
        else:
            proc = asyncio.create_task(self.no.exec())
        try:
            await proc
            self.status.set("done") 
        except:
            rospy.logerr(f"{self} failed!")
            self.status.set("fail")
########################################################
class Group(Template):
    class_name = "Group"  
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
    async def exec(self) -> None:
        await super().exec()          
########################################################
class Task(Template):
    class_name = "Task"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.micros_list = list()
        for exec, subargs in Task.parseMicroList(args):
            micro = constructors_dict[exec](self,exec,subargs)
            self.micros_list.append(micro)
    async def midExec(self) -> None:
        for micro in self.micros_list:
            await micro.exec()
            while Interrupt.queue:
                inter = Interrupt.queue.pop()
                await inter.exec()
    @staticmethod
    def parseMicroList(unparsed:list) -> list:
        for dict in unparsed:
            entry_name = list(dict.keys())[0]
            entry_value = dict[entry_name]
            yield (entry_name, entry_value)
    def __str__(self) -> str:
        return f"{self.name}"
########################################################
class Interrupt(Task):
    class_name = "Interrupt"
    queue = list()
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
    async def midExec(self) -> None:
        for micro in self.micros_list:
            await micro.exec()
    def trigger(self):
        super(Template).trigger()
        Interrupt.queue.append(self)
    @staticmethod
    def parseCond():
        pass
    @staticmethod
    def forceCall(parent, name, args):
        return Interrupt.list[int(args)]
########################################################
class Timer(Template):
    class_name = "Timer"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
    async def midExec(self) -> None:
        pass

##########################################################
class Goto(Template):
    class_name = "Goto"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
    async def midExec(self) -> None:
        pass
class Schedule(Template):
    class_name = "Schedule"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
    async def midExec(self) -> None:
        pass
############################################################
constructors_dict = {  #syntax for route.yaml
        "call":Call,
        "log": Log,
        "move": Move,
        "condition":Condition,
        "group": Group,
        "interrupt": Interrupt.forceCall,
        "interrupt_condition": Interrupt.parseCond,
        "skip": Skip,
        "score": Score,
        "dynamic_call": DynamicCall,
        "sleep": Sleep,
        "goto": Goto,
        "schedule": Schedule
        } 


#####################################
def startCallback(start):
    Flags._execute = start.data
##############
class Manager:
    #Params
    debug = rospy.get_param("~debug", 1)
    #
    file = rospy.get_param("~file", "config/routes/test_route.yaml")
    #
    start_topic = rospy.get_param("~start_topic", "/ebobot/begin")
    #
    start_subscriber = rospy.Subscriber(start_topic, Bool, startCallback)
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
        for interrupt_name in cls.route["interrupts"]:
            pass
        for task_name in cls.route["tasks"]:
            unparsed_list = cls.route["tasks"][task_name]
            new_task = Task(task_name,unparsed_list)
    @staticmethod
    async def exec():
        rospy.sleep(1)
        for task in Task.list:
            proc = asyncio.create_task(task.exec())
            await proc
            if rospy.is_shutdown():
                break
        Flags._execute = 0
##############################
class Flags:
    _execute = 0
def main():
    rospy.on_shutdown(shutdownHook)
    Manager.read()
    if not Manager.route:
        rospy.logerr(f"Route is empty or missing!")
        return
    start_time = rospy.Time.now()
    rospy.logwarn("Parsing route...")
    Manager.parse()
    rospy.logwarn(f"Route parsed in {(rospy.Time.now() - start_time).to_sec()}")
    rate = rospy.Rate(Status.update_rate)
    status_task = Thread(target=Status.update)
    status_task.start()
    while not rospy.is_shutdown():
        if Flags._execute:
            asyncio.run(executeRoute())
        rate.sleep()
async def executeRoute():
    rospy.logwarn(f"Starting route!")
    main_timer = Timer()
    main_task = asyncio.create_task(Manager.exec())
    await main_task
def shutdownHook():
    rospy.signal_shutdown("Task Manager switched off!")
if __name__=="__main__":
    main()