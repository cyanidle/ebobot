import roslib
roslib.load_manifest('ebobot')
import rospy
import actionlib
import yaml
import asyncio
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback, MoveGoal
from ebobot.srv import Servos, ServosRequest, ServosResponse, LcdShow, LcdShowRequest, LcdShowResponse, OhmReaderRequest, OhmReader
from actionlib_msgs.msg import GoalStatus
#from ebobot.srv import Catcher
#
def executer_dict():
    return Execute.dict
def getMoveClient():
    return Move()
class Calls: #Async
    #move_servo = rospy.ServiceProxy("servos_service", Servos)
    def __init__(self, name,execs, static = True):
        rospy.logwarn(f"Initialising a call {name}, {static = }")
        self.executables = []
        self.args = []
        self.parsers = []
        self.static = static
        self.name = name   
        rospy.loginfo(f"Parsing through execs...\n{execs}")
        if static:
            for sub_dict in execs:
                exec_name = list(sub_dict.keys())[0]
                rospy.loginfo(f"Parsing {exec_name =}")
                args = []
                self.executables.append(Execute.exec_dict[exec_name]())
                rospy.loginfo(f"Appended executable {self.executables[-1]}")
                self.parsers.append(Execute.parsers_dict[exec_name])
                rospy.loginfo(f"Appended parser {self.parsers[-1]}")
                for arg in sub_dict[exec_name]:
                    args.append(arg)
                self.args.append(args)
                rospy.loginfo(f"Appended args {self.args[-1]}")
        else:
            for exec in execs:
                rospy.loginfo(f"Parsing {exec =}")
                args = []
                self.executables.append(Execute.exec_dict[exec]())
                rospy.loginfo(f"Appended executable {self.executables[-1]}")
                self.parsers.append(Execute.parsers_dict[exec])
                rospy.loginfo(f"Appended parser {self.parsers[-1]}")
    def __str__(self):
        return f"Call {self.name}:\n###{self.executables = }\n###{self.parsers = }\n###{self.args = }\n"
    def __repr__(self):
        return f"Call {self.name}:\n###{self.executables = }\n###{self.parsers = }\n###{self.args = }\n"
    async def exec(self,args=None):
        "If the call is dynamic, you should pass args to exec() method! Use dict for kwargs of the service"
        rospy.loginfo(f"{self.static = }")
        if self.static:
            proc = asyncio.create_task(self.executeStatic())
        else:
            proc = asyncio.create_task(self.executeDynamic(args))
        return await proc
    async def executeStatic(self):
            resps = []
            for exec, args, parser in zip(self.executables, self.args, self.parsers):
                _corout = exec(parser(args,self.static))
                resps.append(await _corout)
                #sub_calls.append(asyncio.create_task(_corout))
            #resp = await asyncio.gather(*sub_calls)
            rospy.loginfo(f"Executing static {self.name}, responces = {resps}")
            if 1 in resps:
                return 1
            else:
                return 0
    async def executeDynamic(self,args):
        sub_calls = []
        for tup in zip(self.executables, self.parsers):
            exec, parser = tup
            _corout = exec(parser(args,self.static))
            sub_calls.append(asyncio.create_task(_corout))
        resp = await asyncio.gather(*sub_calls)
        rospy.loginfo(f"Executing dynamic {self.name}, responces = {resp}")
        if 1 in resp:
            return 1
        else:
            return 0
    @staticmethod
    def parseServos(args,static:bool):
        parsed = ServosRequest()
        rospy.loginfo(f"Parsing args for servo")
        if static:
            parsed.num = args[0]["num"]
            parsed.state = bool(args[1]["state"])
        else:
            parsed.num = args["num"]
            parsed.state = bool(args["state"])
        return parsed
    @staticmethod
    def parseLcd(args,static):
        parsed = LcdShowRequest()
        if static:
            parsed.num = args["num"]
        else:
            parsed.num = args[0]
        return parsed
    @staticmethod
    def parseOhm(args,static:bool):
        parsed =  OhmReaderRequest()
        if static:
            parsed.num = args[0]["pin"]
        else:
            parsed.num = args["pin"]
        return parsed
    @staticmethod
    def getServoExec():
        return Calls.ServoExec
    @staticmethod
    async def ServoExec(args):
        rospy.sleep(0.2)
        proxy = rospy.ServiceProxy("servos_service", Servos)
        rospy.logwarn(f"Calling servos_service with {args = }")
        return proxy(args).resp
    @staticmethod
    def getLcdExec():
        return Calls.LcdExec
    @staticmethod
    async def LcdExec(args):
        proxy = rospy.ServiceProxy("lcd_service", LcdShow)
        return proxy(args).resp
    @staticmethod
    async def ohmsExec(args):
        proxy = rospy.ServiceProxy("ohm_reader_service", OhmReader)
        resp = round(float(proxy(args).ohms)/1000,2)
        if abs(resp-1) < 0.2:
            return 1
        elif resp < 1:
            return 0
        else:
            return 2 
    @staticmethod
    def getOhmExec():
        return Calls.ohmsExec
async def showPrediction(num):
    """Костыль)))"""
    parsed = LcdShowRequest()
    parsed.num = num
    return await Calls.LcdExec(parsed)
#############
class Execute:
    file = rospy.get_param("/task_manager/calls_file", "config/calls/calls_dict.yaml")
    #/Params
    #Globals
    dict = {}
    raw_dict = {}
    exec_dict = {
        "servos_service":  Calls.getServoExec,
        "prediction_service": Calls.getLcdExec,
        "ohm_reader_service": Calls.getOhmExec
    }
    parsers_dict = {
        "servos_service": Calls.parseServos,
        "prediction_service": Calls.parseLcd,
        "ohm_reader_service": Calls.parseOhm
    }
    @classmethod
    def read(cls):
        rospy.loginfo("Reading file...")
        with open(cls.file, "r") as stream:
            try:
                cls.raw_dict = (yaml.safe_load(stream))
                rospy.loginfo(f"Got dict!")
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")
    @classmethod
    def parse(cls):
        #rospy.loginfo(f"Raw dict = {cls.raw_dict}")
        static = cls.raw_dict["Static"]
        for call_dict in static:
            call_name = list(call_dict.keys())[0]
            rospy.loginfo(f"Parsing {call_name = }")
            try:
                cls.dict[call_name] = Calls(call_name, call_dict[call_name])
            except:
                rospy.logerr(f"Syntax error for static {call_name}! (No available services or duplicate call name)")
        dynamic = cls.raw_dict["Dynamic"]
        for call_dict in dynamic:
            call_name = list(call_dict.keys())[0]
            rospy.logwarn(f"Parsing {call_name = }")
            try:
                cls.dict[call_name] = Calls(call_name, call_dict[call_name], static=False)
            except:
                rospy.logerr(f"Syntax error for dynamic {call_name}! (No available services or duplicate call name)")

class Move:
    def __init__(self,cb):
        "Init a client with a callback function!"
        self.cb = cb
        self.client = actionlib.SimpleActionClient('move', MoveAction)
        rospy.logwarn(f"Waiting for server 'move'...")
        self.client.wait_for_server()
    def setTarget(self,target_pos,target_th):
        rospy.loginfo(f"##Set new goal for {target_pos = }, {target_th = }, with {self.cb = }")
        self.target_pos = target_pos
        self.target_th = target_th
        goal = MoveGoal()
        goal.x = target_pos[1]
        goal.y = target_pos[0]
        goal.theta = target_th
        self.client.send_goal(goal, feedback_cb=self.cb)
    def fetchResult(self):
        return self.client.get_result().result
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

rospy.loginfo(f"Parsing calls from file {Execute.file}...")
Execute.read()
Execute.parse()
rospy.loginfo("Done parsing calls!")



