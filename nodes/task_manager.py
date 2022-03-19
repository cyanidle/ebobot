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
#
from markers import pubMarker
from calls_executer import executer_dict, showPrediction
from calls_executer import Move as move_client_constructor
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
#


def startCallback(start):
    _execute = start.data
########## Subclasses
class Task:
    list =  []
    @staticmethod
    def parseMicroList(unparsed:list) -> list:
        micro_list = []
        for dict in unparsed: ####
            entry_name = list(dict.keys())[0]
            entry_value = dict[entry_name]
            micro_list.append(   (entry_name, entry_value)   )
        return micro_list
    class Microtasks:                #EACH ENTRY'S CONSTRUCTOR MUST RETURN AN OBJECT WITH A EXEC() METHOD
        class Conditions:            #WHICH EXECUTES THE COMMAND AND ONLY THEN RETURNS TO MAIN
            if_list = []             #EXECUTING THE OBJECT SHALL RETURN STATUS (BAD/GOOD/CUSTOM)
            else_list = []
            counter = 0
            def __init__(self,parent,name,args):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = f"{name}_{self.num}"
                self.check_args = args[0]["if"]
                self.yes = self.parseDo(args[1])
                self.no = self.parseElse(args[2])
            def check(self) -> bool:
                return Status.check(self.check_args)
            def parseDo(self,args):
                args = args["do"]
                sub_name = list(args.keys())[0]
                return Task.Microtasks(self,sub_name,args[sub_name])
            def parseElse(self,args):
                args = args["else"]
                sub_name = list(args.keys())[0]
                return Task.Microtasks(self,sub_name,args[sub_name])
            async def exec(self):
                if self.check():
                    proc = asyncio.create_task(self.yes.action.exec())
                else:
                    proc = asyncio.create_task(self.no.action.exec())
                return await proc
            def __str__(self):
                return f"Condition {self.num}"
            def __repr__(self):
                return f"Condition {self.num}"
        class Calls:
            counter = 0
            dyn_counter = 0
            def __init__(self,parent,name,args, dyn = False):
                if not dyn:
                    rospy.loginfo(f"Static call init! {parent = }, {name = }, {args = }")
                    self.parent = parent
                    self.num = type(self).counter
                    type(self).counter += 1
                    self.args = args
                    self.name = f"{name}_{self.num}:{args}"
                    try:
                        self.call = executer_dict()[args]
                    except:
                        raise SyntaxError("No such call name available!")
                    self.curr_status = None
                    Status.add(self)
                else:
                    key = args[0]
                    sub_list = args[1:]
                    pargs = {}
                    for sub_dict in sub_list:
                        pargs = {**pargs, **sub_dict}
                    rospy.loginfo(f"Dynamic call init! {parent = }, {name = }, {pargs = }")
                    self.parent = parent
                    self.num = type(self).dyn_counter
                    type(self).dyn_counter += 1
                    self.args = pargs
                    self.name = f"{name}_{self.num}"
                    self.call = executer_dict()[key]
                    self.curr_status = None
                    Status.add(self)
            @classmethod
            def initDynamic(cls,parent,name,args):
                return cls(parent,name,args,dyn=True)
            async def exec(self):
                rospy.loginfo(f"{self.args = }")
                proc = asyncio.create_task(self.call.exec(self.args))
                try:
                    self.curr_status = await proc
                except:
                    self.curr_status = "bad"
                    rospy.logerr("Service anavailable!")
            def status(self):
                return self.curr_status
            def updateStatus(self):
                pass
            def __str__(self):
                return f"Call {self.num}: {self.name}"
            def __repr__(self):
                return f"Call {self.num}: {self.name}"
        class Move:
            counter = 0
            curr_status = None
            def __init__(self,parent,name,pos:str):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = f"{name}_{self.num}"
                parsed = pos.split("/")
                try:
                    self.pos = (float(parsed[0]),float(parsed[1]))
                    self.th = float(parsed[2])
                except:
                    raise SyntaxError(f"Incorrect move syntax({pos})! Use (x/y/th), th in radians")
            async def exec(self):
                Status.add(self)
                type(self).client.setTarget(self.pos,self.th)
                type(self).client.waitResult()
                self.curr_status = type(self).client.checkResult()
            def status(self):
                return self.curr_status
            def updateStatus(self):
                pass
            @staticmethod
            def cb(fb):
                Status.moves[-1].curr_status = fb.status
            client = move_client_constructor(cb)
            @staticmethod
            def feedback(fb):
                Task.Microtasks.Move.curr_status = fb.status
            def __str__(self):
                return f"Move {self.num}: {self.pos = }"
            def __repr__(self):
                return f"Move {self.num}: {self.pos = }"
        class Logs:
            counter = 0
            def __init__(self,parent,name,args:str):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = f"{name}_{self.num}"
                self.text = args
            async def exec(self):
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
            def updateStatus(self):
                pass
            def __str__(self):
                return f"Log {self.num}: {self.name}"
            def __repr__(self):
                return f"Log {self.num}: {self.name}"
        class Prediction:
            score = 0
            counter = 0
            def __init__(self,parent,name, num):
                self.num = type(self).counter
                type(self).counter += 1
                self.name = f"{name}_{self.num}"
                self.parent = parent
                self.score = num
            def exec(self):
                type(self).score += self.score
                return showPrediction(type(self).score)
        class Skip:
            counter = 0
            def __init__(self,parent,name,num):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = f"{name}_{self.num}"
                self.task_num = int(num)
            async def exec(self):
                Task.list[self.task_num]._skip_flag = 1
            def status(self):
                return "good"
            def updateStatus(self):
                pass
            def __str__(self):
                return f"Skip {self.num}: {self.task_num = }"
            def __repr__(self):
                return f"Skip {self.num}: {self.task_num = }"
        class Sleep:
            counter = 0
            def __init__(self,parent,name,num):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = f"{name}_{self.num}"
                self.time= float(num)
            async def exec(self):
                rospy.sleep(self.time)
            def status(self):
                return "good"
            def updateStatus(self):
                pass
            def __str__(self):
                return f"Sleep_{self.num}: {self.time= }"
            def __repr__(self):
                return f"Sleep_{self.num}: {self.time = }"
        class Together:
            counter = 0
            def __init__(self,parent,name,args):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = f"{name}_{self.num}"
                self.subtask_list = []
                self.micro_list = []
                for sub_dict in args:
                    subtask_name = list(sub_dict.keys())[0]
                    sub_args = sub_dict[subtask_name]
                    self.micro_list.append(Task.Microtasks(self,subtask_name,sub_args)) ##FINISH THIS!!!
            async def exec(self):
                subtasks = []
                for micro in self.micro_list:
                    if Manager.debug:
                        rospy.loginfo(f"Executing {micro} in {self}")
                    task = asyncio.create_task(micro.action.exec())
                    subtasks.append(task)
                await asyncio.gather(*subtasks)            
            def status(self):
                return "good"
            def updateStatus(self):
                pass 
            def __str__(self):
                return f"Together_{self.num}"
            def __repr__(self):
                return f"Together'_{self.num}"
        ############ Microtask
        counter = 0
        def __init__(self,parent,key,args):
            rospy.loginfo(f"Initialising microtask:\ntype = {key}\n{args = }\n{parent = }")
            self.num = type(self).counter
            self.name = key
            self.parent_task = parent
            type(self).counter += 1
            # try:
            self.action = Manager.constructors_dict[key](self,key,args) #parse args correctly, the args of the func are not working!
            # except:
                # raise SyntaxError(f"Incorrect route syntax({key = }, {args = })")
        def __str__(self):
            return f"Microtask {self.num}: {self.name}"
        def __repr__(self):
            return f"Microtask {self.num}: {self.name}"
    ######### Task
    counter = 0
    def __init__(self,name:str, list:list):
        "Inits a new task from dict, parses microtasks and appends them into own (micros_list)"
        self.num = type(self).counter
        type(self).counter += 1
        self._skip_flag = 0
        self.name = f"{name}"
        self.micro_list = []
        for exec_name, args in self.parseMicroList(list):
            rospy.loginfo(f"Parsing {exec_name} with {args = } in {self}...")
            self.micro_list.append(Task.Microtasks(self,exec_name,args)) #micro is a tuple (key, val) for current dict position
        Task.list.append(self)
    async def exec(self):
        if not self._skip_flag:
            for micro in self.micro_list:
                if Manager.debug:
                    rospy.loginfo(f"Executing {micro} in {self}")
                proc = asyncio.create_task(micro.action.exec())
                await proc
                if rospy.is_shutdown():
                    break
        else:
            rospy.loginfo(f"Skipping {self}")
    def __str__(self):
        return f"Task {self.num} ({self.name})"
    def __repr__(self):
        return f"Task {self.num} ({self.name})"
    #pass
######################
class Interrupts(Task):
    list = []
    class Switch:
        counter = 0
        def __init__(self,parent,name,num):
            self.parent = parent
            self.inter_num = int(num)
            self.num = type(self).counter
            type(self).counter += 1
            self.name = f"{name}_{self.num}"
        async def exec(self):
            Task.list[self.inter_num]._skip_flag = 1
        def status(self):
            return "good"
        def updateStatus(self):
            pass
        def __str__(self):
            return f"Interrupt switch {self.num}: {self.inter_num = }"
        def __repr__(self):
            return f"Interrupt switch {self.num}: {self.inter_num = }"
    ########################
    def __init__(self):
        super().__init__(self)
        self.parents_list = []
        Interrupts.list.append(self)
    @staticmethod
    def forceCallParse(parent, _, arg):
        inter = Interrupts.list[arg]
        inter.parents_list.append(parent)
        return inter
    async def exec(self):
        if not self._skip_flag:
            for micro in self.micro_list:
                if Manager.debug:
                    rospy.loginfo(f"Executing {micro} in {self}")
                proc = asyncio.create_task((micro.action.exec()))
                await proc
        else:
            rospy.loginfo(f"Skipping {self}")
    def parseCond(self,arg):
        pass
    @classmethod
    def update(cls):
        pass
########################
class Status:   ### EACH OBJECT WHICH IS ADDED TO STATUS SERVER SHOULD HAVE A STATUS AND UPDATE STATUS METHOD,
    calls = []  ### STATUS RETURNS THE DESIRED VALUE, WHILE UPDATE JUST GETS CALLED EACH STATUS SERVER UPDATE CYCLE
    moves = []
    timers = []
    class Timer:     
        counter = 0
        def __init__(self,main=False) -> None:
            self.main = main
            self.num = type(self).counter
            type(self).counter += 1
            self.time = 0
            self.ros_time = rospy.Time.now()
            Status.add(self)
        def status(self):
            return self.time
        def updateStatus(self):
            self.time = (rospy.Time.now() - self.ros_time).to_sec()
        def __str__(self):
            return f"Timer {self.num} (Main = {self.main})"
        def __repr__(self):
            return f"Timer {self.num} (Main = {self.main})"
    @staticmethod
    def add(obj):
        rospy.loginfo(f"Adding object {obj} to {type(obj)} status tracking list")
        t_list = status_dict[type(obj)]
        t_list.append(obj)
    @staticmethod
    def update():
        rate = rospy.Rate(Manager.update_rate)
        while not rospy.is_shutdown():
            if Manager.debug:
                start_time = rospy.Time.now()
            for obj in Status.calls:
                obj.updateStatus()
            for obj in Status.moves:
                obj.updateStatus()
            for obj in Status.timers:
                obj.updateStatus()
            print (f"Inner timer: {Status.timers[0].time}")
            rate.sleep()
    @staticmethod
    def check(string:str):
        parsed = string.split("/")
        curr = parsed[0]
        name = parsed[1]
        cond = parsed[2]
        if Manager.debug:
                rospy.loginfo(f"Checking type {curr} with {name = } for {cond = }!")
        curr_list = status_check_dict[curr]
        if name in curr_list:
            if cond == curr_list[curr_list.index(name)].status():
                return True
            else:
                return False
        else:
            if Manager.debug:
                rospy.loginfo(f"Name {name} not found in tracking list!")
            return False
####
status_dict = {           #dict for appending from code
        Task.Microtasks.Calls: Status.calls,
        Task.Microtasks.Move: Status.moves,
        Status.Timer: Status.timers
    }    
status_check_dict = {     #dict for syntax in yaml
        "calls": Status.calls,
        "moves": Status.moves,
        "timer": Status.timers
    }  
    
######################
class Manager:

    #Params
    debug = rospy.get_param("~debug", 1)
    #
    update_rate = rospy.get_param("~update_rate", 5)
    file = rospy.get_param("~file", "config/routes/example_route.yaml")
    #
    start_topic = rospy.get_param("~start_topic", "ebobot/begin")
    #
    start_subscriber = rospy.Subscriber(start_topic, Bool, startCallback)
    #
    #/Params
    #Globals
    route = {}
    constructors_dict = {  #syntax for route.yaml
        "call": Task.Microtasks.Calls,
        "log": Task.Microtasks.Logs,
        "move": Task.Microtasks.Move,
        "condition":Task.Microtasks.Conditions,
        "together": Task.Microtasks.Together,
        "interrupt": Interrupts.forceCallParse,
        "skip": Task.Microtasks.Skip,
        "score": Task.Microtasks.Prediction,
        "dynamic_call": Task.Microtasks.Calls.initDynamic,
        "sleep": Task.Microtasks.Sleep
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
        Flags._executing = 0

class Flags:
    _execute = 0
    _executing = 0
    # @staticmethod
    # async def doneCallback():
    #     Flags._execute, Flags._executing = 0, 0
async def main():
    status_task = Thread(target=Status.update)
    status_task.start()
    while not rospy.is_shutdown():
        if Flags._execute and not Flags._executing :
            rospy.logwarn(f"Starting route!")
            Flags._executing = 1
            main_task = asyncio.create_task(Manager.exec())
            await main_task
        rate.sleep()
def shutdownHook():
    rospy.signal_shutdown("Task Manager switched off!")

####
if __name__=="__main__":
    rospy.on_shutdown(shutdownHook)
    Manager.read()
    start_time = rospy.Time.now()
    rospy.logwarn("Parsing route...")
    Manager.parse()
    rospy.logwarn(f"Route parsed in {(rospy.Time.now() - start_time).to_sec()}")
    rate = rospy.Rate(Manager.update_rate)
    #Task.Microtasks.Move.client = move_client_constructor(Task.Microtasks.Move.feedback)
    main_timer = Status.Timer(main=True)
    Flags._execute = 1
    main_task = asyncio.run(main())
    
    
