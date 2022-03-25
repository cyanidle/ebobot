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
    def get(self) -> str:
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
        return str(obj) + obj.status.get()
        #return str(obj.parent.name) + "/" + str(obj.name) + "/"+ str(obj.num) + "/"+ str(obj.status.get())
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
    ############################## MUST be overridden
    @abstractmethod
    def __init__(self,parent,name,args):
        rospy.loginfo(f"Initialising {name} with args {args}, inside of {parent}")
        self.parent = parent
        self._activate_flag = False
        self.num = 0
        if type(parent) == Task:
            [self.num += 1 for obj in self.parent.micros_list if obj.name == self.name]
        else:
            self.num = len(type(self).list)
        type(self).list.append(self)
        self.name = name
        self.status = Status(self)
    @abstractmethod
    async def midExec(self)-> None:
        raise NotImplementedError(self.midExec)
    ############################ Should not be overriden
    async def exec(self)->None:
        self.status.set("executing")
        rospy.loginfo(f"Executing {self}")
        ############### Done before ^^^
        self.midExec()
        ############### Done after main exec
        self._activate_flag = False
        Status.checkDeps(self)
    ####### Not neccessary to override
    def trigger(self) -> None:
        self._activate_flag = True
    def updateStatus(self):
        return self.status.get()  
    def __str__(self) -> str:
        return f"{self.parent.name}/{self.name}/{self.num}"
    def __repr__(self):
        return f"<{type(self).class_name} {self.num}: args - {self.args}>"
########################################################
class Skip(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.task_num = int(args)
    async def midExec(self) -> None:
        Task.list[self.task_num]._skip_flag = 1

##############################################
class Call(Template):
    class_name = "Call"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        try:
            self.call = executer_dict()[args]
        except:
            raise SyntaxError("No such call name available!")
    async def midExec(self) -> None:
        proc = asyncio.create_task(self.call.exec(self.args))
        try:
            self.status.set(await proc)
        except:
            self.status.set("fail")
            rospy.logerr("Service anavailable!")
class DynamicCall(Call):
    class_name = "Dynamic call"
    def __init__(self, parent, name, args):
        super(Template).__init__(parent, name, args)
        key = args[0]
        sub_list = args[1:]
        pargs = {}
        for sub_dict in sub_list:
            pargs = {**pargs, **sub_dict}
        rospy.loginfo(f"Dynamic call init! {parent = }, {name = }, {pargs = }")
        self.args = pargs
        self.call = executer_dict()[key]
########################################################
class Move(Template):
    class_name = "Move client"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        parsed = args.split("/")
        try:
            self.pos = (float(parsed[0]),float(parsed[1]))
            self.th = float(parsed[2])
        except:
            raise SyntaxError(f"Incorrect move syntax({args})! Use (x/y/th), th in radians")
    def midExec(self) -> None:
        type(self).client.setTarget(self.pos,self.th)
        type(self).client.waitResult()
        self.status.set(type(self).client.checkResult()) 
    ####
    @staticmethod
    def cb(fb):
        Move.list[-1].status.set(fb.status) 
    client = move_client_constructor(cb)
########################################################
class Log(Template):
    class_name = "Log"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.text = args    
    def midExec(self) -> None:
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
            self.status.set("fail") 
            return
        self.status.set("done")
########################################################
class Prediction(Template):
    class_name = "Prediction"
    score = 0
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.score = int(args)
    def midExec(self) -> None:
        Prediction.score += self.score
        showPrediction(Prediction.score)
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
class Sleep(Template):
    class_name = "Sleep"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.time = float(args)
    def midExec(self) -> None:
        rospy.sleep(self.time)
########################################################
class Group(Template):
    class_name = "Group"  
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.subtask_list = []
        for sub_dict in args:
            subtask_name = list(sub_dict.keys())[0]
            sub_args = sub_dict[subtask_name]
            self.subtask_list.append(Task(self,subtask_name,sub_args))
    async def midExec(self) -> None:
        subtasks = []
        for micro in self.subtask_list:
            if Manager.debug:
                rospy.loginfo(f"Executing {micro} in {self}")
            task = asyncio.create_task(micro.action.exec())
            subtasks.append(task)
        await asyncio.gather(*subtasks)           
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
        Task.checkForInterrupt()
        for micro in self.micros_list:
            await micro.exec()
            Task.checkForInterrupt()
    @staticmethod
    def checkForInterrupt():
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
        "score": Prediction,
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
