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
#from std_srvs.srv import Empty, EmptyResponse
#
from markers import pubMarker
rospy.logwarn(f"Script won`t start without 'Move' server (Global_planer)")
from calls_executer import executer_dict, showPrediction
rospy.logwarn(f"Server found, calls parsed")
from calls_executer import Move as move_client_constructor
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
#

def startCallback(start):
    Flags._execute = start.data
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
                self.name = name
                self.check_args = args[0]["if"]
                self.yes = self.parseDo(args[1]["do"])
                self.no = self.parseElse(args[2]["else"])
            def check(self) -> bool:
                return Status.check(self.check_args)
            def parseDo(self,args):
                sub_name = list(args.keys())[0]
                return Task.Microtasks(self,sub_name,args[sub_name])
            def parseElse(self,args):
                sub_name = list(args.keys())[0]
                return Task.Microtasks(self,sub_name,args[sub_name])
            async def exec(self):
                if self.check():
                    proc = asyncio.create_task(self.yes.action.exec())
                else:
                    proc = asyncio.create_task(self.no.action.exec())
                try:
                    await proc
                    self.curr_status = "done"
                except:
                    rospy.logerr(f"{self} failed!")
                    self.curr_status = "fail"
                self.who_checked = []
            def __str__(self):
                return f"<Condition {self.num}>"
            def __repr__(self):
                return f"<Condition {self.num}>"
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
                    self.name = name
                    try:
                        self.call = executer_dict()[args]
                    except:
                        raise SyntaxError("No such call name available!")
                    self.curr_status = "init"
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
                    self.curr_status = "init"
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
                    self.curr_status = "fail"
                    rospy.logerr("Service anavailable!")
                finally:
                    self.who_checked = []
            def status(self):
                return self.curr_status
            def updateStatus(self):
                pass
            def __str__(self):
                return f"<Call {self.num}: {self.name}>"
            def __repr__(self):
                return f"<Call {self.num}: {self.name}>"
        class Move:
            counter = 0
            curr_status = None
            def __init__(self,parent,name,pos:str):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = name
                parsed = pos.split("/")
                try:
                    self.pos = (float(parsed[0]),float(parsed[1]))
                    self.th = float(parsed[2])
                    self.curr_status = "init"
                except:
                    raise SyntaxError(f"Incorrect move syntax({pos})! Use (x/y/th), th in radians")
            async def exec(self):
                Status.add(self)
                type(self).client.setTarget(self.pos,self.th)
                type(self).client.waitResult()
                self.curr_status = type(self).client.checkResult()
                self.who_checked = []
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
                return f"<Move {self.num}: {self.pos = }>"
            def __repr__(self):
                return f"<Move {self.num}: {self.pos = }>"
        class Logs:
            counter = 0
            def __init__(self,parent,name,args:str):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = name
                self.text = args
                self.curr_status = "init"
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
                    self.curr_status = "fail"
                self.curr_status = "done"
                self.who_checked = []
            def status(self):
                return self.curr_status
            def updateStatus(self):
                pass
            def __str__(self):
                return f"<Log {self.num}: {self.name}>"
            def __repr__(self):
                return f"<Log {self.num}: {self.name}>"
        class Prediction:
            score = 0
            counter = 0
            def __init__(self,parent,name, num):
                self.num = type(self).counter
                type(self).counter += 1
                self.name = name
                self.parent = parent
                self.score = num
            def exec(self):
                type(self).score += self.score
                self.who_checked = []
        class Skip:
            counter = 0
            def __init__(self,parent,name,num):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = name
                self.task_num = int(num)
            async def exec(self):
                Task.list[self.task_num]._skip_flag = 1
                self.who_checked = []
            def status(self):
                return "good"
            def updateStatus(self):
                pass
            def __str__(self):
                return f"<Skip {self.num}: {self.task_num = }>"
            def __repr__(self):
                return f"<Skip {self.num}: {self.task_num = }>"
        class Sleep:
            counter = 0
            def __init__(self,parent,name,num):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = name
                self.time= float(num)
            async def exec(self):
                rospy.sleep(self.time)
                self.who_checked = []
            def status(self):
                return "good"
            def updateStatus(self):
                pass
            def __str__(self):
                return f"<Sleep_{self.num}: {self.time= }>"
            def __repr__(self):
                return f"<Sleep_{self.num}: {self.time = }>"
        class Together:
            counter = 0
            def __init__(self,parent,name,args):
                self.parent = parent
                self.num = type(self).counter
                type(self).counter += 1
                self.name = name
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
                self.who_checked = []     
            def status(self):
                return "good"
            def updateStatus(self):
                pass 
            def __str__(self):
                return f"<Together {self.num}>"
            def __repr__(self):
                return f"<Together' {self.num}>"
        ############ Microtask
        counter = 0
        def __init__(self,parent,key,args):
            rospy.loginfo(f"Initialising microtask:\ntype = {key}\n{args = }\n{parent = }")
            self.num = type(self).counter
            self.name = key
            self.parent_task = parent
            type(self).counter += 1
            # try:
            self.action = constructors_dict[key](self,key,args) #parse args correctly, the args of the func are not working!
            # except:
                # raise SyntaxError(f"Incorrect route syntax({key = }, {args = })")
        def __str__(self):
            return f"<Microtask {self.num}: {self.name}>"
        def __repr__(self):
            return f"<Microtask {self.num}: {self.name}>"
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
        if type(self) == Task:
            Task.list.append(self)
        self.curr_status = "init"
    async def exec(self):
        self.curr_status = "executing"
        if not self._skip_flag:
            for micro in self.micro_list:
                if Manager.debug:
                    rospy.loginfo(f"Executing {micro} in {self}")
                proc = asyncio.create_task(micro.action.exec())
                await proc
                if len(Interrupts.execute_queue):
                    for inter in Interrupts.execute_queue:
                        sub_proc = asyncio.create_task(inter.exec())
                        await sub_proc
                ###
                if rospy.is_shutdown():
                    break
            self.curr_status = "done"
        else:
            rospy.loginfo(f"Skipping {self}")
            self.curr_status = "skipped"
        self.who_checked = []
    def status(self):
        return self.curr_status
    def updateStatus(self):
        pass 
    def __str__(self):
        return f"<Task {self.num} ({self.name})>"
    def __repr__(self):
        return f"<Task {self.num} ({self.name})>"
    #pass
######################
class Interrupts(Task):
    list = []
    execute_queue = []
    class Switch:
        counter = 0
        def __init__(self,parent,name,num):
            self.parent = parent
            self.inter_num = int(num)
            self.num = type(self).counter
            type(self).counter += 1
            self.name = f"{name}_{self.num}"
            self.curr_status = "init"
            Status.add(self)
        async def exec(self):
            Task.list[self.inter_num]._skip_flag = 1
            self.curr_status = "set"
        def status(self):
            return self.curr_status
        def updateStatus(self):
            pass
        def __str__(self):
            return f"<Interrupt switch {self.num} for inter {self.inter_num}>"
        def __repr__(self):
            return f"<Interrupt switch {self.num} for inter {self.inter_num}>"
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
            rospy.loginfo_once(f"{self} was called, but switched off!")
        self.who_checked = []
    def parseCond(self,args:str):
        pargs = args.split("/")
        self.status_to_check = args
        self.target_status = pargs[-1]
    def status(self):
        return self.curr_status
    def updateStatus(self):
        if (Status.check(self.status_to_check) == self.target_status
        and not self in Interrupts.execute_queue):
            Interrupts.execute_queue.append(self)
    def __str__(self):
        return f"<Interrupt {self.num}>"
    def __repr__(self):
        return f"<Interrupt {self.num}>"
########################
class Status:   ### EACH OBJECT WHICH IS ADDED TO STATUS SERVER SHOULD HAVE A STATUS AND UPDATE STATUS METHOD,
    calls = []  ### STATUS RETURNS THE DESIRED VALUE, WHILE UPDATE JUST GETS CALLED EACH STATUS SERVER UPDATE CYCLE
    moves = []
    timers = []
    interrupts = []
    #
    update_rate = rospy.get_param("~status/update_rate", 5)
    #
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
        rospy.loginfo(f"Adding object {obj} to status tracking list")
        t_list = status_dict[type(obj)]
        t_list.append(obj)
    @staticmethod
    def update():
        rate = rospy.Rate(Status.update_rate)
        while not rospy.is_shutdown():
            for obj in Status.calls:
                obj.updateStatus()
            for obj in Status.moves:
                obj.updateStatus()
            for obj in Status.timers:
                obj.updateStatus()
            for obj in Status.interrupts:
                obj.updateStatus()
            rate.sleep()
    @staticmethod
    def check(string:str,who):
        parsed = string.split("/")
        check_type = parsed[0]
        name = parsed[1]
        num = parsed[2]
        cond = parsed[3]
        if Manager.debug:
                rospy.loginfo(f"Checking type {check_type} with {name = } for {cond = }!")
        curr_list = status_check_dict[check_type]
        filtered  = []
        filtered = [_obj for _obj in curr_list if _obj.name == name]
        try:
            obj = filtered[num]
        except:
            if Manager.debug:
                rospy.loginfo(f"Name {name} not found in tracking list!")
            return False
        if not obj.who_checked:
            obj.who_checked = [who]
        else:
            obj.who_checked.append(who)
        if cond == obj.status() and not who in obj.who_checked:
            return True
        else:
            return False
######################
class Manager:

    #Params
    debug = rospy.get_param("~debug", 1)
    #
    
    file = rospy.get_param("~file", "config/routes/example_route.yaml")
    #
    start_topic = rospy.get_param("~start_topic", "/ebobot/begin")
    #
    start_subscriber = rospy.Subscriber(start_topic, Bool, startCallback)
    #
    #/Params
    #Globals
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
#####################
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
    null_timer = Status.Timer()
    while not rospy.is_shutdown():
        if Flags._execute:
            asyncio.run(executeRoute())
        rate.sleep()
async def executeRoute():
    rospy.logwarn(f"Starting route!")
    main_timer = Status.Timer(main=True)
    main_task = asyncio.create_task(Manager.exec())
    await main_task
def shutdownHook():
    rospy.signal_shutdown("Task Manager switched off!")
#################################
status_dict = {           #dict for appending from code
        Task.Microtasks.Calls: Status.calls,
        Task.Microtasks.Move: Status.moves,
        Status.Timer: Status.timers,
        Interrupts: Status.interrupts
    }    
status_check_dict = {     #dict for syntax in yaml
        "calls": Status.calls,
        "moves": Status.moves,
        "timers": Status.timers,
        "interrupts": Status.interrupts
    }  
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
#################################
if __name__=="__main__":
    main()
    
    
    
