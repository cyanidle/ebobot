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
from std_srvs.srv import Empty, EmptyResponse
#
from markers import pubMarker
from calls_executer import executer_dict, showPrediction
from calls_executer import Move as move_client_constructor
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
#
from abc import ABC, abstractmethod


class Status:
    update_rate = rospy.get_param("~/status/update_rate", 1)
    amplify_rate_for_move = rospy.get_param("~/status/amplify_rate_for_move", 1)
    #
    list = []
    deps_dict = {}
    _cycle_rate = rospy.Rate(update_rate)
    def __init__(self, parent):
        self._status = "init"
        self.parent = parent
    def update(self):
        rospy.loginfo(f"Updating status of {self.parent}")
        self._status = self.parent.updateStatus()
    def get(self) -> str:
        return self._status
    def set(self, text):
        self._status = str(text)
    @staticmethod
    def updateCycle():
        while not rospy.is_shutdown():
            if "Timer" in Manager.obj_dict.keys():
                for timer in Manager.obj_dict["Timer"]:
                    timer.status.update()
            else:
                rospy.logwarn(f"No timers found!")
            Status._cycle_rate.sleep()
    @staticmethod
    def checkDeps(obj):
        obj_str = Status.getRawString(obj)
        rospy.logwarn(f"Checking deps of {obj}: {obj_str}")
        if obj_str in Status.deps_dict.keys():
            for dep in Status.deps_dict[obj_str]:
                dep.trigger()
                rospy.logwarn(f"{dep} was triggered!")
    @staticmethod
    def getRawString(obj):
        return f"{obj.rawString()}/{obj.status.get()}"
        #return str(obj.parent.name) + "/" + str(obj.name) + "/"+ str(obj.num) + "/"+ str(obj.status.get())
    @staticmethod
    def registerDependency(obj, condition:str):
        rospy.loginfo(f"Adding dependecy of {obj} from condition {condition}")
        if condition in Status.deps_dict.keys():
            Status.deps_dict[condition].append(obj) 
        else:
            Status.deps_dict[condition] = [obj]
    @staticmethod
    def clearDependency(obj, condition:str):
        if condition in Status.deps_dict.keys():
            Status.deps_dict[condition].remove(obj) 
        else:
            Status.deps_dict[condition] = []
#######################################################
class Template(ABC):
    list = []
    ############################## MUST be overridden
    @abstractmethod
    def __init__(self,parent,name,args):
        rospy.loginfo(f"Initialising {name} with args {args}, inside of {parent}")
        self.parent = parent
        self._activate_flag = False
        self.num = 0
        self.name = name
        if type(parent) == Task:
            for obj in self.parent.micros_list:
                if obj.name == self.name:
                    self.num += 1
        else:
            self.num = len(type(self).list)
        cls_name = type(self).__name__
        if cls_name in Manager.obj_dict.keys():
            Manager.obj_dict[cls_name].append(self)
        else:
            Manager.obj_dict[cls_name] = [self]
        self.status = Status(self)
    @abstractmethod
    async def midExec(self)-> None:
        raise NotImplementedError(self.midExec)
    ############################ Should not be overriden
    async def exec(self)->None:
        await Task.checkForInterrupt()
        self.status.set("executing")
        rospy.loginfo(f"{self}")
        ############### Done before ^^^
        await self.midExec()
        ############### Done after main exec
        await Task.checkForInterrupt()
        self._activate_flag = False
        Status.checkDeps(self)
    ####### Not neccessary to override
    def trigger(self) -> None:
        self._activate_flag = True
    def updateStatus(self) -> str:
        return self.status.get()  
    def __str__(self) -> str:
        return f"<{type(self).__name__} {self.num}|status:{self.status.get()}>"
    def rawString(self):
        return f"{self.parent.name}/{self.name}/{self.num}"
        
########################################################
class Skip(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.task_num = int(args)
    async def midExec(self) -> None:
        Manager.obj_dict["Task"][self.task_num]._skip_flag = 1

##############################################
class Call(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.args = None
        try:
            self.call = executer_dict()[args]
        except:
            raise SyntaxError(f"No such call name available ({name}), check calls_dict!")
    async def midExec(self) -> None:
        proc = asyncio.create_task(self.call.exec(self.args))
        self.status.set(await proc)
        # try:
        #     self.status.set(await proc)
        # except:
        #     self.status.set("fail")
        #     rospy.logerr("Service anavailable!")
class DynamicCall(Call):
    def __init__(self, parent, name, args):
        super(Call,self).__init__(parent, name, args)
        key = args[0]
        sub_list = args[1:]
        pargs = {}
        for sub_dict in sub_list:
            pargs = {**pargs, **sub_dict}
        rospy.loginfo(f"Dynamic call init! {parent}: {name}, {pargs}")
        self.args = pargs
        self.call = executer_dict()[key]
########################################################
def mv_cb(fb):
    rospy.loginfo(f"Move server status: {fb.status}")
    Move.curr_obj.status.set(fb.status) 
class Move(Template):
    curr_obj = None
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        parsed = args.split("/")
        try:
            self.pos = (float(parsed[1]),float(parsed[0]))
            self.th = float(parsed[2])
        except:
            raise SyntaxError(f"Incorrect move syntax({args})! Use (x/y/th), th in radians")
    async def midExec(self) -> None:
        Move.curr_obj = self
        type(self).client.setTarget(self.pos,self.th)
        _stat = self.status.get()
        _ended = 0
        while not _ended and not rospy.is_shutdown():
            #await Task.checkForInterrupt()
            _stat = type(self).client.checkResult()
            #rospy.logwarn(f"{_stat = }| {type(_stat) = }")
            rospy.loginfo(f"Move server feedback {_stat}")
            if _stat == 2 or _stat == 4:
                if await Task.checkForInterrupt():
                    await asyncio.sleep(0.1)
                    type(self).client.setTarget(self.pos,self.th)
            else:
               _ended = 1
               self.status.set(_stat)
            await asyncio.sleep((1/Status.update_rate) / Status.amplify_rate_for_move)
        self.status.set(type(self).client.fetchResult()) 
    client = move_client_constructor(mv_cb)
    

    
########################################################
class Log(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.text = args    
    async def midExec(self) -> None:
        text = self.text
        pref = text[:2]
        if pref == "L:":
            rospy.loginfo(f"Log:{text[2:]}")
        elif pref == "W:":
            rospy.logwarn(f"Log:{text[2:]}")
        elif pref == "E:":
            rospy.logerr(f"Log:{text[2:]}")
        else:
            rospy.logerr(f"(Incorrect log prefix!) {text}")
            self.status.set("fail") 
            return
        self.status.set("done")
########################################################
class Prediction(Template):
    score = 0
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.score = int(args)
    async def midExec(self) -> None:
        Prediction.score += self.score
        try:
            await showPrediction(Prediction.score)
        except:
            rospy.logerr(f"Service for {self} unavailable!")
########################################################
class Condition(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        check_args = args[0]["if"]
        self.yes = self.parse(args[1]["do"])
        self.no = self.parse(args[2]["else"])
        Status.registerDependency(self,check_args)
    def parse(self,args):
        for sub_name, sub_args in Task.parseMicroList(args):
            return constructors_dict[sub_name](self,sub_name,sub_args)
    def trigger(self):
        self._activate_flag = True
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
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.time = float(args)
    async def midExec(self) -> None:
        await asyncio.sleep(self.time)
        self.status.set("done")
########################################################
class Group(Template):
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
    def __init__(self, name, args):
        super().__init__(Manager, name, args)
        self.micros_list = list()
        self._fail_flag = 0
        #self.name = f"{type(self).__name__} '{self.name}'"
        for exec, subargs in Task.parseMicroList(args):
            micro = constructors_dict[exec](self,exec,subargs)
            self.micros_list.append(micro)
    async def midExec(self) -> None:
        rospy.logwarn(f"Micros in task {self} - {self.micros_list}")
        for micro in self.micros_list:
            await micro.exec()
            if self._fail_flag:
                self.status.set("fail")
                break
        if not self._fail_flag:
            self.status.set("done")
    @staticmethod
    async def checkForInterrupt():
        rospy.loginfo(f"Checking for interrupts")
        _return = 0
        if Interrupt.queue:
            _return = 1
        while Interrupt.queue:
            inter = Interrupt.queue.pop()
            await inter.exec()
        return _return
    @staticmethod
    def parseMicroList(unparsed:list) -> list:
        #rospy.loginfo(f"Parsing list {unparsed}")
        for dict in unparsed:
            entry_name = list(dict.keys())[0]
            entry_value = dict[entry_name]
            #print(f"{dict}, {entry_name}, {entry_value}")
            yield (entry_name, entry_value)
    def __str__(self) -> str:
        return f"{self.name}"
    def rawString(self): #Used to get status
        return f"tasks/{self.name}"
########################################################
class Interrupt(Template):
    queue = list()
    def __init__(self, name, args):
        super().__init__(Manager, name, args)
        self.micros_list = list()
        num_of_conds = 0
        #self.name = f"{type(self).__name__} '{self.name}'"
        for exec, subargs in Task.parseMicroList(args):
            micro = constructors_dict[exec](self,exec,subargs)
            self.micros_list.append(micro)
            if exec == "interrupt_condition":
                num_of_conds += 1
        self.micros_list = self.micros_list[num_of_conds:]
    async def midExec(self) -> None:
        rospy.logwarn(f"Micros in interrupt {self} - {self.micros_list}")
        for micro in self.micros_list:
            await micro.exec()
    def trigger(self):
        Interrupt.queue.append(self)
    @staticmethod
    def parseCond(parent, name, args):
        Status.registerDependency(parent, args)
    @staticmethod
    def forceCall(parent, name, args):
        return Interrupt.list[int(args)]
    def __str__(self) -> str:
        return f"{self.name}"
    def rawString(self): #Used to get status
        return f"interrupts/{self.name}"
########################################################
class Timer(Template):
    name = "timer"
    def __init__(self, name):
        super().__init__(Timer, name, name)
        self.time = 0 
        self.status.set("0")
        self.last_time = rospy.Time.now()
    async def midExec(self) -> None:
        pass
    def delete(self):
        Timer.list.remove(self)
    def updateStatus(self):
        #print (f"{self.time = }")
        #print(f"{(rospy.Time.now() - self.last_time).to_sec() = }")
        self.time += round((rospy.Time.now() - self.last_time).to_sec())
        self.last_time = rospy.Time.now()
        if not self.status.get() == str(self.time):
            self.status.checkDeps(self) 
        return str(self.time)
    def __str__(self) -> str:
        return f"{self.name}|status: {self.status.get()}"
    def rawString(self):
        return f"timer/{self.name}"
##########################################################
class Goto(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self._task_name = args
    async def midExec(self) -> None:
        if not "Task" in Manager.obj_dict.keys():
            return
        for task in Manager.obj_dict["Task"]:
            if task.name == self._task_name:
                Manager.current_task = self.num
                Flags._goto = True
                return
        rospy.logerr(f"Jump to {self._task_name} fail! (No such task)!")
class Schedule(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self._interrupt_name = args
    async def midExec(self) -> None:
        if not "Interrupt" in Manager.obj_dict.keys():
            return
        for inter in Manager.obj_dict["Interrupt"]:
            if inter.name == self._interrupt_name:
                inter.trigger()
        rospy.logerr(f"Interrupt {self._interrupt_name} trigger failed! (No such interrupt)!")
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
        "schedule_interrupt": Schedule,
        "new_timer": Timer
        } 


#####################################
def startCallback(start):
    Flags._execute = start.data
##############
class Manager:
    current_task = 0
    obj_dict = {}
    name = "Tasks"
    #Params
    debug = rospy.get_param("~debug", 1)
    #
    file = rospy.get_param("~file", "config/routes/test_route.yaml")
    #
    start_topic = rospy.get_param("~start_topic", "/ebobot/begin")
    #
    start_subscriber = rospy.Subscriber(start_topic, Bool, startCallback)
    route = {}
    rate = rospy.Rate(Status.update_rate)
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
            unparsed_list = cls.route["interrupts"][interrupt_name]
            #rospy.logerr(f"{unparsed_list = }")
            new_inter = Interrupt(interrupt_name,unparsed_list)
        for task_name in cls.route["tasks"]:
            unparsed_list = cls.route["tasks"][task_name]
            #rospy.logerr(f"{unparsed_list = }")
            new_task = Task(task_name,unparsed_list)
    @staticmethod
    async def exec():
        await asyncio.sleep(0.5)
        #rospy.logerr(f"{Task.list = }")
        while not rospy.is_shutdown() and Flags._execute:
            await Task.checkForInterrupt()
            if Manager.current_task<len(Manager.obj_dict["Task"]):
                task = Manager.obj_dict["Task"][Manager.current_task]
                proc = asyncio.create_task(task.exec())
                await proc
                if Flags._goto:
                    Flags._goto = False
                else:
                    Manager.current_task += 1
            else:
                rospy.logwarn("No tasks left!")
            await asyncio.sleep(0.1)
        Manager.current_task = 0
        Flags._execute = 0
        Manager.rate.sleep()
##############################
class Flags:
    _execute = 0
    _goto = False
def main():
    rospy.on_shutdown(shutdownHook)
    Manager.read()
    rospy.loginfo(f"Got dict!")
    if not Manager.route:
        rospy.logerr(f"Route is empty or missing!")
        return
    start_time = rospy.Time.now()
    rospy.logwarn("Parsing route...")
    Manager.parse()
    rospy.logwarn(f"Route parsed in {(rospy.Time.now() - start_time).to_sec()}")
    rospy.loginfo(f"{Manager.obj_dict}")
    rospy.loginfo("Waiting for start topic...")
    
    status_task = Thread(target=Status.updateCycle)
    status_task.start()
    while not rospy.is_shutdown():
        if Flags._execute:
            asyncio.run(executeRoute())
        
##################
async def executeRoute():
    rospy.logwarn(f"Starting route!")
    main_timer = Timer("main")
    main_task = asyncio.create_task(Manager.exec())
    await main_task
##################
def shutdownHook():
    rospy.signal_shutdown("Task Manager switched off!")
##################
if __name__=="__main__":
    main()
