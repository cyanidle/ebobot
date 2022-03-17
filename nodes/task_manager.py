#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
import yaml
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
    def parseMicroList(list:list) -> list:
        micro_list = []
        for dict in list: ####
            entry_name = list(dict)[0]
            entry_value = dict[entry_name]
            micro_list.append(   (entry_name, entry_value)   )
        return micro_list
    class Microtasks:                #EACH ENTRY'S CONSTRUCTOR MUST RETURN AN OBJECT WITH A EXEC() METHOD
        class Conditions:            #WHICH EXECUTES THE COMMAND AND ONLY THEN RETURNS TO MAIN
            if_list = []             #EXECUTING THE OBJECT SHALL RETURN STATUS (BAD/GOOD/CUSTOM)
            else_list = []
            counter = 0
            def __init__(self,arg):
                self.num = type(self).counter
                type(self).counter += 1
                items = arg.items()
                self.check_args = items[0]
                self.yes = self.parseDo(items[1])
                self.no = self.parseElse(items[2])
                return self
            def check(self) -> bool:
                return Status.check(self.check_args)
            def parseDo(self,args):
                return Task.Microtasks.getExec(args)
            def parseElse(self,args):
                return Task.Microtasks.getExec(args)
            async def exec(self):
                if self.check():
                    self.yes.exec()
                else:
                    self.no.exec()
            def __str__(self):
                return f"Condition {self.num}"
            def __repr__(self):
                return f"Condition {self.num}"
        class Calls:
            counter = 0
            def __init__(self,name,args:tuple):
                self.num = type(self).counter
                type(self).counter += 1
                self.args = args
                self.name = f"{name}{self.num}"
                self.call = executer_dict()[name]
                self.curr_status = None
                Status.add(self,type(self))
                return self
            async def exec(self):
                self.curr_status = self.call(self.args)
            def status(self):
                return self.curr_status
            def statusUpdate(self):
                pass
            def __str__(self):
                return f"Call {self.num}: {self.name}"
            def __repr__(self):
                return f"Call {self.num}: {self.name}"
        class Move:
            counter = 0
            client = move_client_constructor()
            def __init__(self,pos):
                self.num = type(self).counter
                type(self).counter += 1
                self.pos = pos
                return self
            async def exec(self):
                Status.add(self)
                self.curr_status = self.call(self.args)
            def status(self):
                return self.curr_status
            def statusUpdate(self):
                pass
            def __str__(self):
                return f"Move {self.num}: {self.pos = }"
            def __repr__(self):
                return f"Move {self.num}: {self.pos = }"
        class Logs:
            counter = 0
            def __init__(self,args:str):
                self.num = type(self).counter
                type(self).counter += 1
                self.text = args
                return self
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
            def statusUpdate(self):
                pass
        class Prediction:
            score = 0
            def __init__(self, num):
                self.score = num
                return self
            def exec(self):
                type(self).score += self.score
                return showPrediction(type(self).score)
        class Skip:
            counter = 0
            def __init__(self,num):
                self.task_num = num
                self.num = type(self).counter
                type(self).counter += 1
                return self
            async def exec(self):
                Task.list[self.task_num]._skip_flag = 1
            def status(self):
                return "good"
            def statusUpdate(self):
                pass
            def __str__(self):
                return f"Skip {self.num}: {self.task_num = }"
            def __repr__(self):
                return f"Skip {self.num}: {self.task_num = }"
        class Together:
            def __init__(self,args):
                self.num = type(self).counter
                type(self).counter += 1
                self.subtask_list = []
                self.micro_list = []
                for subtask_name in args:
                    unparsed_list = args[subtask_name]
                    for name, args in Task.parseMicroList(subtask_name,unparsed_list):
                        self.micro_list.append(Task.Microtasks.getExec(name,args)) ##FINISH THIS!!!
            async def exec(self):
                subtasks = []
                for micro in self.micro_list:
                    if Manager.debug:
                        rospy.loginfo(f"Executing {micro} in {self}")
                        task = asyncio.create_task(micro.action.exec)
                    subtasks.append(task)
                await asyncio.gather(*subtasks)
                
            def status(self):
                return "good"
            def statusUpdate(self):
                pass 
            def __str__(self):
                return f"'Together' call {self.num}"
            def __repr__(self):
                return f"'Together' call {self.num}"
        ############ Microtask
        counter = 0
        def __init__(self,key,args):
            rospy.loginfo(f"Initialising microtask {key = } {args = }")
            self.num = type(self).counter
            self.name = key
            type(self).counter += 1
            self.action = Manager.constructors_dict[key](args)
            return self
        @staticmethod
        def getExec(key,args): #val in the dict!
            micro = Task.Microtasks(key,args) #each constructor 
            return micro                      #should return an executable object
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
        self.name = f"{name}{self.num}"
        self.micro_list = []
        for name, args in self.parseMicroList(name,args):
            rospy.loginfo(f"Parsing {name} with {args = }...")
            self.micro_list.append(Task.Microtasks.getExec(name,args)) #micro is a tuple (key, val) for current dict position
        Task.list.append(self)
    async def exec(self):
        if not self._skip_flag:
            for micro in self.micro_list:
                if Manager.debug:
                    rospy.loginfo(f"Executing {micro} in {self}")
                asyncio.run(micro.action.exec())
        else:
            rospy.loginfo(f"Skipping {self}")
    def __str__(self):
        return f"Task {self.num}: {self.name}"
    def __repr__(self):
        return f"Task {self.num}: {self.name}"
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
        def __init__(self) -> None:
            self.num = type(self).counter
            type(self).counter += 1
            self.time = 0
            self.ros_time = rospy.Time.now()
            Status.add(self)
        def status(self):
            return self.time
        def updateStatus(self):
            self.time = (rospy.Time.now() - self.ros_time).to_sec()
    @staticmethod
    def add(obj):
        rospy.loginfo(f"Adding object {obj} to {type(obj)} status tracking list")
        t_list = status_dict[type(obj)]
        t_list.append(obj)
    @staticmethod
    def update():
        rospy.loginfo_once("Updating status for tracking lists...")
        if Manager.debug:
            start_time = rospy.Time.now()
        for obj in Status.calls:
            obj.updateStatus()
        for obj in Status.moves:
            obj.updateStatus()
        for obj in Status.timers:
            obj.updateStatus()
        if Manager.debug:
            rospy.loginfo_once(f"Updating status done in {(rospy.Time.now() - start_time).to_sec()}")
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
status_dict = dict = {           #dict for appending from code
        Task.Microtasks.Calls: Status.calls,
        Task.Microtasks.Move: Status.moves,
        Status.Timer: Status.timers
    }    
status_check_dict = dict = {     #dict for syntax in yaml
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
    file = rospy.get_param("~file", "/config/routes/route1.yaml")
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
        "score": Task.Microtasks.Prediction
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
    def exec():
        for task in Task.list:
            asyncio.run(task.exec())



async def doneCallback():
    _execute, _executing = 0, 0
def main():
    Manager.read()
    start_time = rospy.Time.now()
    rospy.logwarn("Parsing route...")
    Manager.parse()
    rospy.logwarn(f"Route parsed in {(rospy.Time.now() - start_time).to_sec()}")
    timer = Status.Timer()
    rate = rospy.Rate(Manager.update_rate)
    main_task = asyncio.create_task(Task.exec())
    cb_task = doneCallback()
    main_task.add_done_callback(cb_task)
    while not rospy.is_shutdown():
        if not _executing and _execute:
            _executing = 1
            asyncio.run(main_task)
        Status.update()
        rate.sleep()

 

####
if __name__=="__main__":
    _execute = 0
    _executing = 0
    main()
