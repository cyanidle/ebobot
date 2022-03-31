import roslib
import subprocess
roslib.load_manifest('ebobot')
import rospy
import actionlib
import yaml
import asyncio
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback, MoveGoal
from ebobot.srv import (Servos, ServosRequest, ServosResponse, LcdShow,
 LcdShowRequest, LcdShowResponse, PinReaderRequest, PinReader)
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty, EmptyRequest
#from ebobot.srv import Catcher
#
def executer_dict():
    return Execute.dict
def getMoveClient():
    return Move()
class Calls: #Async
    #move_servo = rospy.ServiceProxy("servos_service", Servos)
    def __init__(self, name,execs, static = True):
        rospy.logwarn(f"Initialising a call {name}, static:{static}")
        self.executables = []
        self.args = []
        self.parsers = []
        self.static = static
        self.name = name   
        rospy.loginfo(f"Parsing through execs...\n{execs}")
        if static:
            for sub_dict in execs:
                exec_name = list(sub_dict.keys())[0]
                rospy.loginfo(f"Parsing {exec_name}")
                self.executables.append(Execute.exec_dict[exec_name]())
                rospy.loginfo(f"Appended executable {self.executables[-1]}")
                self.parsers.append(Execute.parsers_dict[exec_name])
                rospy.loginfo(f"Appended parser {self.parsers[-1]}")
                pargs = {}
                print (sub_dict[exec_name])
                for sub_dict_args in sub_dict[exec_name]:
                    pargs = {**pargs, **sub_dict_args}
                self.args.append(pargs)
                rospy.loginfo(f"Appended args {pargs} to {self.args}")
        else:
            for exec in execs:
                rospy.loginfo(f"Parsing {exec}")
                args = []
                self.executables.append(Execute.exec_dict[exec]())
                rospy.loginfo(f"Appended executable {self.executables[-1]}")
                self.parsers.append(Execute.parsers_dict[exec])
                rospy.loginfo(f"Appended parser {self.parsers[-1]}")
    def __str__(self):
        return f"Call {self.name}:\n###{self.executables}\n###{self.parsers}\n###{self.args}\n"
    def __repr__(self):
        return f"Call {self.name}:\n###{self.executables}\n###{self.parsers}\n###{self.args}\n"
    async def exec(self,args=None):
        "If the call is dynamic, you should pass args to exec() method! Use dict for kwargs of the service"
        rospy.loginfo(f"static:{self.static}")
        if self.static:
            proc = asyncio.create_task(self.executeStatic())
        else:
            proc = asyncio.create_task(self.executeDynamic(args))
        return await proc
    async def executeStatic(self):
        resps = []
        try:
            for exec, args, parser in zip(self.executables, self.args, self.parsers):
                _corout = exec(parser(args))
                resps.append(await _corout)
            rospy.loginfo(f"Executed static {self.name}, responces = {resps}")
            if 1 in resps:
                return "fail"
            else:
                return "done"
        except:
            rospy.logerr(f"Call {self.name} unavailable!")
            return "fail"
    async def executeDynamic(self,args):
        sub_calls = []
        try:
            for tup in zip(self.executables, self.parsers):
                exec, parser = tup
                _corout = exec(parser(args))
                sub_calls.append(asyncio.create_task(_corout))
            resp = await asyncio.gather(*sub_calls)
            rospy.loginfo(f"Executing dynamic {self.name}, responces = {resp}")
            if 1 in resp:
                return "fail"
            else:
                return "done"
        except:
            rospy.logerr(f"Call {self.name} unavailable!")
            return "fail"
    @staticmethod
    def parseServos(args):
        parsed = ServosRequest()
        rospy.loginfo(f"Parsing args for servo")
        parsed.num = args["num"]
        parsed.state = int(args["state"])
        return parsed
    @staticmethod
    def parseLcd(args):
        parsed = LcdShowRequest()
        parsed.num = args["num"]
        return parsed
    @staticmethod
    def parseOhm(args):
        parsed = PinReaderRequest()
        parsed.digital = False
        parsed.write = False
        parsed.pullup = True
        parsed.pin = args["pin"]
        return parsed
    @staticmethod
    def getServoExec():
        return Calls.ServoExec
    @staticmethod
    async def ServoExec(args):
        rospy.sleep(0.2)
        proxy = rospy.ServiceProxy("servos_service", Servos)
        rospy.logwarn(f"Calling servos_service with args:{args}")
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
        proxy = rospy.ServiceProxy("pin_reader_service", PinReader)
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
    ####
    @staticmethod
    def getAdjExec():
        return rospy.ServiceProxy("adjust_pos_service", Empty)
    @staticmethod
    def adjExec(args):
        proxy = rospy.ServiceProxy("adjust_pos_service", Empty)
        proxy(args)
        return 0
    @staticmethod
    def parseAdj(_place):
        return EmptyRequest()
    ####
    @staticmethod
    def getPinExec():
        return Calls.pinExec
    @staticmethod
    def pinExec(args):
        proxy = rospy.ServiceProxy("pin_reader_service", PinReader)
        proxy(args)
        return 0
    @staticmethod
    def parsePin(args):
        parsed =  PinReaderRequest()
        parsed.pin = args["pin"]
        parsed.digital = args["digital"]
        parsed.write = args["write"]
        parsed.pullup = args["pullup"]
        parsed.value = args["value"]
        return parsed
##############################################
async def showPrediction(num):
    """Костыль)))"""
    parsed = LcdShowRequest()
    parsed.num = num
    return await Calls.LcdExec(parsed)
#############
class Execute:
    file = rospy.get_param("~calls_file", "config/calls/calls_dict.yaml")
    #/Params
    #Globals
    dict = {}
    raw_dict = {}
    exec_dict = {
        "servos_service":  Calls.getServoExec,
        "prediction_service": Calls.getLcdExec,
        "ohm_reader_service": Calls.getOhmExec,
        "adjust_pos_service": Calls.getAdjExec,
        "pin_service": Calls.getPinExec
    }
    parsers_dict = {
        "servos_service": Calls.parseServos,
        "prediction_service": Calls.parseLcd,
        "ohm_reader_service": Calls.parseOhm,
        "adjust_pos_service": Calls.parseAdj,
        "pin_service": Calls.parsePin
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
            rospy.loginfo(f"Parsing {call_name}")
            try:
                cls.dict[call_name] = Calls(call_name, call_dict[call_name])
            except:
                rospy.logerr(f"Syntax error for static {call_name}! (No available services or duplicate call name)")
        dynamic = cls.raw_dict["Dynamic"]
        for call_dict in dynamic:
            call_name = list(call_dict.keys())[0]
            rospy.logwarn(f"Parsing {call_name}")
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
        rospy.loginfo(f"##Set new goal for {target_pos}, {target_th}")
        self.target_pos = target_pos
        self.target_th = target_th
        goal = MoveGoal()
        goal.x = target_pos[1]
        goal.y = target_pos[0]
        goal.theta = target_th
        self.client.send_goal(goal, feedback_cb=self.cb)
    def fetchResult(self):
        return self.client.get_result()
    def checkResult(self)->str:
        "ACTIVE = 0, SUCCESS = 1, ABORTED = 2, LOST = 3, ELSE = 4"
        state = self.client.get_state()
        if state == GoalStatus.ACTIVE:
            return "executing"
        elif state == GoalStatus.SUCCEEDED:
            return "done"
        elif state == GoalStatus.ABORTED:
            return "fail"
        elif state == GoalStatus.LOST:
            return "fail"
        else:
            return "executing"
    def waitResult(self):
        self.client.wait_for_result()

rospy.loginfo(f"Parsing calls from file {Execute.file}...")
Execute.read()
Execute.parse()
rospy.loginfo("Done parsing calls!")
#out = subprocess.run("echo", "1.1.1.1",text=True)

#out = int(subprocess.run(["ifconfig"], ["|"], ["sed"] ,["-En"], ["'s/127.0.0.1//;s/.*inet"], ["(addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p'",text=True]).stdout.split(".")[-1])
#ip = int(out.stdout.split(".")[-1])
try:
    asyncio.run(showPrediction(7770))
except:
    rospy.logwarn("Arduino disconnected!")



