import roslib
roslib.load_manifest('ebobot')
import rospy
import actionlib
import yaml
import asyncio
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback, MoveGoal
from ebobot.srv import Servos, ServosRequest, ServosResponse, Lcd_show, Lcd_showRequest, Lcd_showResponse
from actionlib_msgs.msg import GoalStatus
#from ebobot.srv import Catcher
#
def executer_dict():
    return Execute.dict
def getMoveClient():
    return Move()
class Calls: #Async
    #move_servo = rospy.ServiceProxy("servos_service", Servos)
    @staticmethod
    def getServoExec():
        return rospy.ServiceProxy("servos_service", Servos)
    @staticmethod
    def getLcdExec():
        return rospy.ServiceProxy("lcd_service", Lcd_show)
    def __init__(self, name,execs, static = True):
        print(f"Initialising a call {name}")
        self.executables = []
        self.args = []
        self.parsers = []
        self.static = static
        self.name = name   
        print(f"Parsing through execs...\n{execs}")
        if static:
            for sub_dict in execs:
                exec_name = list(sub_dict.keys())[0]
                print(f"Parsing {exec_name =}")
                args = []
                self.executables.append(Execute.exec_dict[exec_name]())
                print(f"Appended executable {self.executables[-1]}")
                self.parsers.append(Execute.parsers_dict[exec_name])
                print(f"Appended parser {self.parsers[-1]}")
                for arg in sub_dict[exec_name]:
                    args.append(arg)
                self.args.append(args)
                print(f"Appended args {self.args[-1]}")
        else:
            for exec in execs:
                print(f"Parsing {exec =}")
                args = []
                self.executables.append(Execute.exec_dict[exec]())
                print(f"Appended executable {self.executables[-1]}")
                self.parsers.append(Execute.parsers_dict[exec])
                print(f"Appended parser {self.parsers[-1]}")
    def __str__(self):
        return f"Call {self.name}:\n###{self.executables = }\n###{self.parsers = }\n###{self.args = }\n"
    def __repr__(self):
        return f"Call {self.name}:\n###{self.executables = }\n###{self.parsers = }\n###{self.args = }\n"
    async def exec(self,args=None):
        "If the call is dynamic, you should pass args to exec() method! Use list, each index corresponds to a service, under each index is a tuple"
        if self.static == True:
            asyncio.run(self.executeStatic())
        else:
            asyncio.run(self.executeDynamic(args))
    async def executeStatic(self):
            sub_calls = []
            for exec, args, parser in zip(self.executables, self.args, self.parsers):
                sub_calls.append(asyncio.create_task(exec(parser(args,self.static))))
            resp = await asyncio.gather(*sub_calls)
            rospy.loginfo(f"Executing static {self.name}, responces = {resp}")
            if 1 in resp:
                return 1
            else:
                return 0
    async def executeDynamic(self,args):
        sub_calls = []
        for num,tup in enumerate(zip(self.executables, self.parsers)):
            exec, parser = tup
            sub_calls.append(asyncio.create_task(exec(parser(args[num],self.static))))
        resp = await asyncio.gather(*sub_calls)
        rospy.loginfo(f"Executing dynamic {self.name}, responces = {resp}")
        if 1 in resp:
            return 1
        else:
            return 0
    @staticmethod
    def parseServos(args,static):
        parsed = ServosRequest()
        if static:
            parsed.num = args["num"]
            parsed.state = args["state"]
        else:
            parsed.num = args[0]
            parsed.state = args[1]
        return parsed
    @staticmethod
    def parseLcd(args,static):
        parsed = Lcd_showRequest()
        if static:
            parsed.num = args["num"]
        else:
            parsed.num = args[0]
        return parsed
def showPrediction(num):
    parsed = Lcd_showRequest()
    parsed.num = num
    return Calls.lcd_show(parsed)
#############
class Execute:
    file = rospy.get_param("/task_manager/calls_file", "config/calls/calls_dict.yaml")
    #/Params
    #Globals
    dict = {}
    raw_dict = {}
    exec_dict = {
        "servos_service":  Calls.getServoExec,
        "prediction_service": Calls.getLcdExec
    }
    parsers_dict = {
        "servos_service": Calls.parseServos,
        "prediction_service": Calls.parseLcd
    }
    @classmethod
    def read(cls):
        print("Reading file...")
        with open(cls.file, "r") as stream:
            try:
                cls.raw_dict = (yaml.safe_load(stream))
                print(f"Got dict!")
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")
    @classmethod
    def parse(cls):
        #print(f"Raw dict = {cls.raw_dict}")
        static = cls.raw_dict["Static"]
        for call_dict in static:
            call_name = list(call_dict.keys())[0]
            print(f"Parsing {call_name = }")
            try:
                cls.dict[call_name] = Calls(call_name, call_dict[call_name])
            except:
                print(f"Syntax error for static {call_name}! (No available services or duplicate call name)")
        dynamic = cls.raw_dict["Dynamic"]
        for call_dict in dynamic:
            call_name = list(call_dict.keys())[0]
            print(f"Parsing {call_name = }")
            try:
                cls.dict[call_name] = Calls(call_name, call_dict[call_name], static=False)
            except:
                print(f"Syntax error for dynamic {call_name}! (No available services or duplicate call name)")

class Move:
    def __init__(self,cb):
        "Init a client with a callback function!"
        self.cb = cb
        self.client = actionlib.SimpleActionClient('move', MoveAction)
        print(f"Waiting for server 'move'...")
        self.client.wait_for_server()
    def setTarget(self,target_pos,target_th):
        print(f"##Set new goal for {target_pos = }, {target_th = }, with {self.cb = }")
        self.target_pos = target_pos
        self.target_th = target_th
        goal = MoveGoal()
        goal.x = target_pos[1]
        goal.y = target_pos[0]
        goal.theta = target_th
        self.client.send_goal(goal, feedback_cb=self.cb)
    def fetchResult(self):
        return self.client.get_result()
    def checkResult(self)->int:
        "ACTIVE = 0, SUCCESS = 1, ABORTED = 2, LOST = 3, ELSE = 4"
        state = self.client.get_state()
        if state == GoalStatus.ACTIVE:
            return 2
        elif state == GoalStatus.SUCCEEDED:
            return 0
        elif state == GoalStatus.ABORTED:
            return 1
        elif state == GoalStatus.LOST:
            return 3
        else:
            return 4
    def waitResult(self):
        self.client.wait_for_result()

print(f"Parsing calls from file {Execute.file}...")
Execute.read()
Execute.parse()
print("Done parsing calls!")
print(f"{Execute.dict = }")



