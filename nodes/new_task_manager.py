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
from ebobot.srv import ChangeCost, ChangeCostRequest
#
from abc import ABC, abstractmethod
##
rospy.sleep(1)
#####################################
def startCallback(start):
    rospy.logerr(f"MANAGER: Got new start command {start.data}")
    Flags._execute = 0
    # while Flags._busy:
    #    rospy.sleep(0.03)
    if start.data == 1:
        try:
            asyncio.run(showPrediction(7771))
        except:
            rospy.logwarn("No lcd found!")
        if Manager.debug:
            rospy.logwarn(f"Route 1!")
        Flags._current_route_num = 1
        _parse()
    elif start.data == 2:
        try:
            asyncio.run(showPrediction(7772))
        except:
            rospy.logwarn("No lcd found!")
        if Manager.debug:
            rospy.logwarn(f"Route 2!")
        Flags._current_route_num = 2
        _parse()
    elif start.data == 0:
        parse(10+Flags._current_route_num)
        Flags._test_routes = 1
        Flags._countdown = 1
        rospy.logwarn(f"MANAGER: Test Start!")
        for n in range(3,0,-1):
            try:
                asyncio.run(showPrediction(n))
                if not Flags._test_routes:
                    break
            except:
                rospy.logwarn("No lcd found!")
            rospy.sleep(1)
        Flags._countdown = 0
        if Flags._test_routes:
            asyncio.run(showPrediction(0))
            _run()    
    elif start.data == 3:
        Flags._test_routes = 0
        if Flags._countdown:
            rospy.logwarn(f"MANAGER: Fast Start!")
            _parse(Flags._current_route_num)
        else:
            rospy.logwarn(f"MANAGER: Route Start!")
        _run()
    else:
        try:
            asyncio.run(showPrediction(9999))
        finally:
            rospy.logerr("MANAGER: Unexpected start data!")
##################################################
def _parse():
    parse(Flags._test_routes*10 + Flags._current_route_num)
def _run():
    rospy.loginfo("MANAGER: Setting flag...")
    Flags._busy = 1
    Flags._execute = 1

##################################################
class Status:
    update_rate = rospy.get_param("~/status/update_rate", 1)
    reduce_rate_for_move = rospy.get_param("~/status/reduce_rate_for_move", 1)
    #
    list = []
    deps_dict = {}
    _cycle_rate = rospy.Rate(update_rate)
    def __init__(self, parent):
        self._status = "init"
        self.parent = parent
    def update(self):
        #if Manager.debug:
        #    rospy.loginfo(f"Updating status of {self.parent}")
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
            #else:
                #if Manager.debug:
                #    #rospy.logwarn(f"No timers found!")
            Status._cycle_rate.sleep()
    @staticmethod
    def checkDeps(obj):
        obj_str = Status.getRawString(obj)
        if Manager.debug:
            rospy.loginfo(f"Checking deps of {obj}: {obj_str}")
        if obj_str in Status.deps_dict.keys():
            for dep in Status.deps_dict[obj_str]:
                dep.trigger()
                if Manager.debug:
                    rospy.logwarn(f"{dep} was triggered!")
    @staticmethod
    def getRawString(obj):
        return f"{obj.rawString()}/{obj.status.get()}"
        #return str(obj.parent.name) + "/" + str(obj.name) + "/"+ str(obj.num) + "/"+ str(obj.status.get())
    @staticmethod
    def registerDependency(obj, condition:str):
        if Manager.debug:
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
        if Manager.debug:
            if type(self) == Task:
                rospy.loginfo(f"Initialising {name} with {len(args)} subtasks")
            else:
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
            self.num = len(Manager.obj_dict[type(self).__name__])-1
        ###
        self.status = Status(self)
    @abstractmethod
    async def midExec(self)-> None:
        raise NotImplementedError(self.midExec)
    ############################ Should not be overriden
    async def exec(self)->None:
        await Task.checkForInterrupt()
        self.status.set("executing")
        if Manager.debug:
            rospy.loginfo(f"{self}")
        ############### Done before ^^^
        await self.midExec()
        ############### Done after main exec
        if Flags._update_prediction:
            await showPrediction(Prediction.score)
        await Task.checkForInterrupt()
        self._activate_flag = False
        if Manager.debug:
            rospy.logwarn(f"Executable done! {self}")
        Status.checkDeps(self)
    ####### Not neccessary to override
    def trigger(self) -> None:
        self._activate_flag = True
    def updateStatus(self) -> str:
        return self.status.get()  
    def __repr__(self) -> str:
        return f"<{type(self).__name__} {self.num}|{self.parent.name}/{self.name}/{self.num}>"
    def __str__(self) -> str:
        return f"<{type(self).__name__} {self.num}|status:{self.status.get()}>"
    def rawString(self):
        return f"{self.parent.name}/{self.name}/{self.num}"
        
########################################################
class Skip(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        if args == "all":
            self.task_num = 0
            self.all_flag = 1
        else:
            self.all_flag = 0
            self.task_num = int(args)
    async def midExec(self) -> None:
        if self.all_flag:
            for task in Manager.obj_dict["Task"]:
                task._skip_flag = 1
        else:
            Manager.obj_dict["Task"][self.task_num]._skip_flag = 1
########################################################
class ChangeCostClass(Template):
    proxy = rospy.ServiceProxy("change_cost_service", ChangeCost)
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.cost = args
    async def midExec(self) -> None:
        try:
            ChangeCostClass.proxy(ChangeCostRequest(self.cost))
        except:
            rospy.logerr("Change cost unavailable!")
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
        # if Manager.debug:    
        #   rospy.logerr("Service anavailable!")
class DynamicCall(Call):
    def __init__(self, parent, name, args):
        super(Call,self).__init__(parent, name, args)
        key = args[0]
        sub_list = args[1:]
        pargs = {}
        for sub_dict in sub_list:
            pargs = {**pargs, **sub_dict}
        if Manager.debug:
            rospy.loginfo(f"Dynamic call init! {parent}: {name}, {pargs}")
        self.args = pargs
        self.call = executer_dict()[key]
########################################################
def mv_cb(fb):
    if Manager.debug:
        rospy.loginfo(f"Move server status: {fb.status}")
    Move.curr_obj.status.set(fb.status) 
class Move(Template):
    curr_obj = None
    def __str__(self) -> str:
        return f"<{type(self).__name__} {self.num}|pos:{self.pos}|status:{self.status.get()}>"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        rospy.logerr(f"INITTING MOVE! NUMBER OF MOVES = {len(Manager.obj_dict['Move'])}|POS = {args}")
        parsed = args.split("/")
        try:
            self.pos = (float(parsed[1]),float(parsed[0]))
            self.th = float(parsed[2])
        except:
            raise SyntaxError(f"Incorrect move syntax({args})! Use (x/y/th), th (rotation) in radians")
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
                    type(self).client.cancel_goal()
                    await asyncio.sleep(0.1)
                    type(self).client.setTarget(self.pos,self.th)
            else:
               _ended = 1
               self.status.set(_stat)
               if _stat == "fail":
                   self.parent._fail_flag = 1
            await asyncio.sleep((0.05) * Status.reduce_rate_for_move)
        self.status.set(type(self).client.fetchResult()) 
    client = move_client_constructor(mv_cb)
    

    
########################################################
class Log(Template):
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.text = args    
    async def midExec(self) -> None:
        text = self.text
        pref = text.split(":")[0]
        if pref == "L":
            rospy.loginfo(f"Log:{text[2:]}")
        elif pref == "W":
            rospy.logwarn(f"Log:{text[2:]}")
        elif pref == "E":
            rospy.logerr(f"Log:{text[2:]}")
        elif pref == "VAR":
            var = Variable.find(text[4:])
            if not var:
                rospy.logerr(f"No such variable available ({text[4:]})")
                self.status.set("fail")
            rospy.logwarn(f"Variable {var}")
        else:
            rospy.logerr(f"(Incorrect log prefix!) {text}")
            self.status.set("fail") 
            return
        self.status.set("done")
########################################################
def predictionCB(num):
    Prediction.score+=num.data
    Flags._update_prediction = 1
class Prediction(Template):
    score = 0
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.score = int(args)
    async def midExec(self) -> None:
        if not self.parent._fail_flag:
            Prediction.score += self.score
            if not Manager.prediction_master:
                Manager.prediction_pub.publish(Int8(self.score))
        try:
            await showPrediction(Prediction.score)
        except:
            if Manager.debug:
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
            if Manager.debug:
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
        if Manager.debug:
            rospy.logwarn(f"Micros in task {self} - {self.micros_list}")
        for micro in self.micros_list:
            await micro.exec()
            if self._fail_flag or not Flags._execute or rospy.is_shutdown():
                self.status.set("fail")
                break
        if not self._fail_flag:
            self.status.set("done")
    @staticmethod
    async def checkForInterrupt():
        #if Manager.debug:
        #    rospy.loginfo(f"Checking for interrupts")
        _return = 0
        while len(Interrupt.queue):
            _return = 1
            inter = Interrupt.queue.pop()
            await inter.exec()
        return _return
    @staticmethod
    def parseMicroList(unparsed:list) -> list:
        #if Manager.debug:
        #    rospy.loginfo(f"Parsing list {unparsed}")
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
        if Manager.debug:
            rospy.logwarn(f"Micros in interrupt {self} - {self.micros_list}")
        for micro in self.micros_list:
            await micro.exec()
            if self._fail_flag or not Flags._execute or rospy.is_shutdown():
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
            if Manager.debug:
                rospy.logerr(f"No such interrupt {args}")
        except:
            if Manager.debug:
                rospy.logerr(f"Interrupt force called when list is empty!")
    def __str__(self) -> str:
        return f"{self.name}"
    def rawString(self): #Used to get status
        return f"interrupts/{self.name}"
########################################################
class Variable(Template):
    name = "Variable"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        self.value = int(args)
        self.status.set(self.value)
    def set(self, num: int):
        self.value = num
        self.status.set(self.value)
        Status.checkDeps(self)
    @staticmethod
    def find(str):
        if "Variable" in Manager.obj_dict.keys():
            for var in Manager.obj_dict["Variable"]:
                if var.name == str:
                    return var
            rospy.logerr("Find var called with zero init variables!")
            return None
        else:
            rospy.logerr("Find var called with zero init variables!")
            return None
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
        var = Variable.find(parsed[2])
        if var:
            self._value = var.value
        else:
            self._value = int(parsed[2])
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
            if Manager.debug:
                rospy.logerr(f"Value change ({self._action}) not implemented!")
    async def midExec(self) -> None:
        #try:
        for var in Manager.obj_dict["Variable"]:
            if var.name == self._var:
                var.set(self.apply(var.value))
                self.status.set("done")
                return
        # except:
        #     if Manager.debug:
        #         rospy.logerr("No variables available")
        #     self.status.set("fail")
########################################################
class StopRoute(Template):
    def __init__(self, parent,name,args):
        super().__init__(parent, name, args)
        self.log = args
    async def midExec(self) -> None:
        rospy.logerr(f"ROUTE ENDED (LOG:{self.log})")
        Manager.reset()
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
        if type(args) == str:
            self._type = "name"
        elif type(args) == int:
            self._type = "num"
        self._task_name = args
    async def midExec(self) -> None:
        if self._type != "name":
            Manager.current_task = self._task_name
            Flags._goto = True
            return
        if not "Task" in Manager.obj_dict.keys():
            return
        for task in Manager.obj_dict["Task"]:
            if task.name == self._task_name:
                Manager.current_task = self.num
                Flags._goto = True
                return
        if not "Variable" in Manager.obj_dict.keys():
            return
        for var in Manager.obj_dict["Variable"]:
            if var.name == self._task_name:
                Manager.current_task = int(var.value)
                Flags._goto = True
                return
        if Manager.debug:
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
        if Manager.debug:
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
        "change_var": ChangeVar,
        "change_cost": ChangeCostClass,
        "route_stop": StopRoute
        } 



##############
class Manager:
    current_task = 0
    obj_dict = {}
    name = "Tasks"
    #Params
    debug = rospy.get_param("~debug", 1)
    if debug:
        rospy.logerr(f"Manager uses debug!")
    #
    file1 = rospy.get_param("~file1", "config/routes/test_route1.yaml")
    file2 = rospy.get_param("~file2", "config/routes/test_route2.yaml")
    test_file1 = rospy.get_param("~test_file1", "config/routes/test_route1.yaml")
    test_file2 = rospy.get_param("~test_file2", "config/routes/test_route2.yaml")
    #
    prediction_master = rospy.get_param("~prediction_master", 1)
    prediction_topic = rospy.get_param("~prediction_topic", "/ebobot/new_prediction")
    start_topic = rospy.get_param("~start_topic", "ebobot/begin")
    ########
    if prediction_master:
        rospy.Subscriber(prediction_topic, Int8, predictionCB)
    else:
        prediction_pub = rospy.Publisher(prediction_topic, Int8)
    #
    curr_file = test_file2
    start_subscriber = rospy.Subscriber(start_topic, Int8, startCallback)
    route = {}
    rate = rospy.Rate(Status.update_rate)
    @classmethod
    def read(cls):
        with open(cls.curr_file, "r") as stream:
            try:
                cls.route = (yaml.safe_load(stream))
            except yaml.YAMLError as exc:
                if Manager.debug:
                    rospy.logerr(f"Loading failed ({exc})")
    @classmethod
    def parse(cls):
        if "interrupts" in cls.route.keys():
            for interrupt_name in cls.route["interrupts"]:
                unparsed_list = cls.route["interrupts"][interrupt_name]
                new_inter = Interrupt(interrupt_name,unparsed_list)
        else:
            if Manager.debug:
                rospy.logwarn("No interrupts found in route file!")
        if "tasks" in cls.route.keys():
            for task_name in cls.route["tasks"]:
                unparsed_list = cls.route["tasks"][task_name]
                new_task = Task(task_name,unparsed_list)
            ###
            if "Move" in cls.obj_dict.keys():
                _color_step = 1/len(Manager.obj_dict["Move"])
                pubMarker((0,0),0,1,frame_name="task_moves", deletall=1)
                for num, _mv in enumerate(Manager.obj_dict["Move"]):
                    pubMarker(_mv.pos,num,300,frame_name="task_moves",
                    type="cube",size=0.05,g=1-((num) * _color_step),r=1,b=1-((num) * _color_step),debug=1,add=1)
                    ###
        else:
            if Manager.debug:
                rospy.logerr("NO TASKS IN ROUTE FILE!")
        if "variables" in cls.route.keys():
            for var_name in cls.route["variables"]:
                print (f"variable {var_name}")
                var_val = cls.route["variables"][var_name]
                new_var = Variable(Variable, var_name, var_val)
        else:
            if Manager.debug:
                rospy.logwarn("No variables found!")
    @staticmethod
    def reset():
        rospy.logwarn(f"MANAGER: Reset called!")
        Flags._execute = 0
        Manager.route = {}
        Manager.obj_dict = {}
        Prediction.score = 0
    @staticmethod
    async def exec():
        if Manager.debug:
            rospy.logwarn(f"Starting route!")
        main_timer = Timer(Timer,"main","main")
        main_timer._allow_flag = 1
        _done = 0
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
                if Manager.debug:
                    rospy.logwarn_once("No tasks left!")
                if not _done:
                    rospy.logwarn(f"MANAGER: Route done test = {Flags._test_routes}")
                    _done = 1   
                    _busy = 0
                    if Flags._test_routes:
                        Flags._execute = 0
                        parse(Flags._current_route_num)
            await asyncio.sleep(0.05)
        Manager.current_task = 0
        if Flags._test_routes:
            parse(Flags._current_route_num)
        return 0
##############################
class Flags:
    _execute = 0
    _test_routes = 1
    _current_route_num = 2
    _update_prediction = 0
    _goto = False
    _busy = 0
    _countdown = 0
def parse(route = 11):
    Manager.reset() #pls fix
    if route == 11:
        Manager.curr_file = Manager.test_file1
    elif route == 12:
        Manager.curr_file = Manager.test_file2
    elif route == 1:
        Manager.curr_file = Manager.file1
    elif route == 2:
        Manager.curr_file = Manager.file2
    else:
        if Manager.debug:
            rospy.logerr("Unavailable route called")
    Manager.read()
    if Manager.debug:
        rospy.loginfo(f"Got dict!")
    if not Manager.route:
        if Manager.debug:
            rospy.logerr(f"Route is empty or missing!")
        return
    start_time = rospy.Time.now()
    if Manager.debug:
        rospy.logwarn("Parsing route...")
    Manager.parse()
    if Manager.debug:
        rospy.logwarn(f"Route parsed in {(rospy.Time.now() - start_time).to_sec()}")
    #if Manager.debug:
    #    rospy.loginfo(f"{Manager.obj_dict}")
    if Manager.debug:
        rospy.loginfo("Waiting for start topic...")
    
def main():
    _parse()
    while not rospy.is_shutdown():
        if Flags._execute:
            asyncio.run(Manager.exec())
        rospy.sleep(0.025)
        # if Flags._execute:
        #     asyncio.run(executeRoute())
        # if Manager.debug:
        #     rospy.loginfo("Waiting for start topic...")
        # rospy.sleep(0.1)
    
    

##################
def shutdownHook():
    rospy.signal_shutdown("Task Manager switched off!")
##################
if __name__=="__main__":
    status_task = Thread(target=Status.updateCycle)
    status_task.start()
    rospy.on_shutdown(shutdownHook)
    main()
