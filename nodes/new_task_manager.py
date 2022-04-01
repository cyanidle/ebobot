#!/usr/bin/env python3
from math import floor
import roslib
roslib.load_manifest('ebobot')
import rospy
import yaml
from threading import Thread
import asyncio
rospy.init_node("task_manager")
#
from std_msgs.msg import Int8
from std_srvs.srv import Empty, EmptyResponse
#
from markers import pubMarker
from calls_executer import executer_dict, showPrediction
from calls_executer import Move as move_client_constructor
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
#
from abc import ABC, abstractmethod
##
rospy.sleep(1)
#####################################
def startCallback(start):
    Flags._execute = 0
    rospy.logwarn(Manager.route)
    if Flags._test_routes:
        if start.data == 1:
            asyncio.run(showPrediction(7771))
            rospy.logwarn(f"Parsing test_route1!")
            Flags._current_route_num = 1
            rospy.sleep(0.5)  
            parse(11)
        elif start.data == 2:
            asyncio.run(showPrediction(7772))
            rospy.logwarn(f"Parsing test_route2!")
            Flags._current_route_num = 2
            rospy.sleep(0.5)
            parse(12)
        elif start.data == 0:
            Flags._test_routes = 0
            rospy.sleep(1)
            for n in range(3,0,-1):
                asyncio.run(showPrediction(n))
                rospy.sleep(1)
            Flags._execute = 1
            rospy.logwarn(f"Executing test route!")
        else:
            rospy.logerr("Incorrect start sequence!")
            asyncio.run(showPrediction(9999))
    else:
        if start.data == 9:
            asyncio.run(showPrediction(1000 + Flags._current_route_num))
            rospy.logwarn(f"Parsing route{Flags._current_route_num}!")
            rospy.sleep(0.5)
            parse(Flags._current_route_num)
        elif start.data == 1:
            asyncio.run(showPrediction(1001))
            rospy.logwarn(f"Parsing route1!")
            Flags._current_route_num = 1
            rospy.sleep(0.5)
            parse(1)
        elif start.data == 2:
            asyncio.run(showPrediction(1002))
            rospy.logwarn(f"Parsing route2!")
            Flags._current_route_num = 2
            rospy.sleep(0.5)
            parse(2)
        elif start.data == 3:
            Flags._test_routes = 1
            Flags._execute = 1
            rospy.logwarn(f"Executing chosen route!")
        else:
            rospy.logerr("Incorrect start sequence!")
            asyncio.run(showPrediction(9999))
            Flags._test_routes = 1
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
        ###
        cls_name = type(self).__name__
        if cls_name in Manager.obj_dict.keys():
            Manager.obj_dict[cls_name].append(self)
        else:
            Manager.obj_dict[cls_name] = [self]
        ###
        if type(parent) == Task or type(parent) == Group:
            for obj in self.parent.micros_list:
                if obj.name == self.name:
                    self.num += 1
        else:
            self.num = len(Manager.obj_dict[type(self).__name__])
        ###
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
        rospy.logwarn(f"Executable done! {self}")
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
        while not _ended and not rospy.is_shutdown() and Flags._execute:
            _stat = type(self).client.checkResult()
            rospy.loginfo(f"Move server feedback {_stat}")
            if _stat == "executing":
                if await Task.checkForInterrupt():
                    await asyncio.sleep(0.1)
                    type(self).client.setTarget(self.pos,self.th)
            else:
               _ended = 1
               self.status.set(_stat)
               if _stat == "fail":
                   self.parent._fail_flag = 1
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
        self.micros_list = []
        self.group_name = self.name
        self.name = f"{super().rawString()}"
        for sub_dict in args:
            subtask_name = list(sub_dict.keys())[0]
            sub_args = sub_dict[subtask_name]
            self.micros_list.append(constructors_dict[subtask_name](self,subtask_name,sub_args))
    async def midExec(self) -> None:
        subtasks = []
        for micro in self.micros_list:
            if Manager.debug:
                rospy.loginfo(f"Executing {micro} in {self}")
            task = asyncio.create_task(micro.exec())
            subtasks.append(task)
        resps = await asyncio.gather(*subtasks)
        if 1 in resps or "fail" in resps:
            self.status.set("fail")
        else:
            self.status.set("done")
    def rawString(self):
        return f"{self.parent.name}/{self.group_name}/{self.num}"      
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
            if self._fail_flag or not Flags._execute:
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
        self._fail_flag = 0
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
            if self._fail_flag or not Flags._execute:
                self.status.set("fail")
                break
        if not self._fail_flag:
            self.status.set("done")
    def trigger(self):
        Interrupt.queue.append(self)
    @staticmethod
    def parseCond(parent, name, args):
        Status.registerDependency(parent, args)
    @staticmethod
    def forceCall(parent, name, args):
        try:
            for inter in Manager.obj_dict["Interrupt"]:
                if inter.name == args:
                    return inter
            rospy.logerr(f"No such interrupt {args}")
        except:
            rospy.logerr(f"Interrupt force called when list is empty!")
    def __str__(self) -> str:
        return f"{self.name}"
    def rawString(self): #Used to get status
        return f"interrupts/{self.name}"
########################################################
class Variable(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.value = int(args)
        self.status.set(self.value)
    def set(self, num: int):
        self.value = num
        Status.checkDeps(self)
    def midExec(self) -> None:
        pass
    def rawString(self):
        return f"var/{self.name}"
########################################################
class ChangeVar(Template): 
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        parsed = args.split("/")
        self._var = parsed[0]
        self._action = parsed[1]
        self._value = parsed[2]
    def apply(self, val: int):
        if self._action == "add":
            return val + self._value
        elif self._action == "sub":
            return val - self._value
        elif self._action == "divide":
            return val / self._value
        elif self._action == "multiply":
            return val * self._value
        else:
            rospy.logerr(f"Value change ({self._action}) not implemented!")
    def midExec(self) -> None:
        try:
            for var in Manager.obj_dict["Variable"]:
                if var.name == self._var:
                    var.set(self.apply(var.value))
            self.status.set("done")
        except:
            rospy.logerr("No variables available")
########################################################
class Timer(Template):
    name = "timer"
    def __init__(self, parent,name,args):
        super().__init__(parent, name, args)
        self.time = 0 
        self.status.set("0")
        self.last_time = rospy.Time.now()
        self._allow_flag = False
    async def midExec(self) -> None:
        self._allow_flag = True
    def delete(self):
        Timer.list.remove(self)
    def updateStatus(self):
        if self._allow_flag:
            #print (f"{self.time = }")
            #print(f"{(rospy.Time.now() - self.last_time).to_sec() = }")
            self.time += round((rospy.Time.now() - self.last_time).to_sec())
            self.last_time = rospy.Time.now()
            if not self.status.get() == str(self.time):
                self.status.checkDeps(self) 
            return str(self.time)
        else:
            return "0"
    def __str__(self) -> str:
        return f"{self.name}|status: {self.status.get()}"
    def rawString(self):
        return f"timers/{self.name}"
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
        self._task_name = args
    async def midExec(self) -> None:
        if not "Task" in Manager.obj_dict.keys():
            self.status.set("fail")
            return
        for task in Manager.obj_dict["Task"]:
            if task.name == self._task_name:
                await task.exec()
                self.status.set(task.status.get())
                return 
        rospy.logerr(f"Task {self._task_name} trigger failed! (No such task)!")
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
        "schedule_task": Schedule,
        "timer": Timer,
        "change_var": ChangeVar
        } 



##############
class Manager:
    current_task = 0
    obj_dict = {}
    name = "Tasks"
    #Params
    debug = rospy.get_param("~debug", 1)
    #
    file1 = rospy.get_param("~file1", "config/routes/test_route.yaml")
    file2 = rospy.get_param("~file2", "config/routes/test_route.yaml")
    test_file1 = rospy.get_param("~test_file1", "config/routes/test_route.yaml")
    test_file2 = rospy.get_param("~test_file2", "config/routes/test_route.yaml")
    #
    start_topic = rospy.get_param("~start_topic", "/ebobot/begin")
    #
    curr_file = test_file1
    start_subscriber = rospy.Subscriber(start_topic, Int8, startCallback)
    route = {}
    rate = rospy.Rate(Status.update_rate)
    @classmethod
    def read(cls):
        with open(cls.curr_file, "r") as stream:
            try:
                cls.route = (yaml.safe_load(stream))
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")
    @classmethod
    def parse(cls):
        if "interrupts" in cls.route.keys():
            for interrupt_name in cls.route["interrupts"]:
                unparsed_list = cls.route["interrupts"][interrupt_name]
                new_inter = Interrupt(interrupt_name,unparsed_list)
        else:
            rospy.logwarn("No interrupts found in route file!")
        if "tasks" in cls.route.keys():
            for task_name in cls.route["tasks"]:
                unparsed_list = cls.route["tasks"][task_name]
                new_task = Task(task_name,unparsed_list)
            ###
            _color_step = 1/len(Manager.obj_dict["Move"])
            pubMarker((0,0),0,1,frame_name="task_moves", deletall=1)
            for num, _mv in enumerate(Manager.obj_dict["Move"]):
               pubMarker(_mv.pos,num,40,frame_name="task_moves",
               type="cube",size=0.05,g=0.5,r=((num+1) * _color_step),b=0.5,debug=1,add=1)
            ###
        else:
            rospy.logerr("NO TASKS IN ROUTE FILE!")
        if "variables" in cls.route.keys():
            for var_name in cls.route["variables"]:
                var_val = cls.route["variables"][var_name]
                new_var = Variable(Variable, var_name, var_val)
        else:
            rospy.logwarn("No variables found!")
    @staticmethod
    def reset():
        Manager.route.clear()
        Manager.obj_dict.clear()
        Prediction.score = 0
    @staticmethod
    async def exec():
        await asyncio.sleep(0.2)
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
                if not Flags._test_routes:
                    startCallback(Int8(9))
            await asyncio.sleep(0.1)
        Manager.current_task = 0
        Flags._execute = 0
        Manager.rate.sleep()
##############################
class Flags:
    _execute = 0
    _test_routes=1
    _current_route_num = 1
    _goto = False
def parse(route = 0):
    Manager.reset()
    if route == 11:
        Manager.curr_file = Manager.test_file1
    elif route == 12:
        Manager.curr_file = Manager.test_file2
    elif route == 1:
        Manager.curr_file = Manager.file1
    elif route == 2:
        Manager.curr_file = Manager.file2
    else:
        rospy.logerr("Unavailable route called")
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
    
def main():
    parse()
    while not rospy.is_shutdown():
        if Flags._execute:
            asyncio.run(executeRoute())
        rospy.loginfo("Waiting for start topic...")
        rospy.sleep(1/Status.update_rate)
    
    
        
##################
async def executeRoute():
    rospy.logwarn(f"Starting route!")
    main_timer = Timer(Timer,"main","main")
    main_timer._allow_flag = 1
    await Manager.exec()
##################
def shutdownHook():
    rospy.signal_shutdown("Task Manager switched off!")
##################
if __name__=="__main__":
    status_task = Thread(target=Status.updateCycle)
    status_task.start()
    rospy.on_shutdown(shutdownHook)
    main()
