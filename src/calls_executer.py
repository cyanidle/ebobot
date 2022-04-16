import roslib
#import subprocess
roslib.load_manifest('ebobot')
import rospy
import actionlib
import yaml
import asyncio
from functools import partial
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback, MoveGoal
from ebobot.srv import ChangeCost, ChangeCostRequest
from ebobot.srv import (Servos, ServosRequest, ServosResponse, LcdShow,
 LcdShowRequest, LcdShowResponse, PinReaderRequest, PinReader)
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse, SetBool, SetBoolResponse, SetBoolRequest
#from ebobot.srv import Catcher
#
def executer_dict():
    return Execute.dict
def getMoveClient():
    return Move()
def reprExec(_,*,obj):
    asyncio.run(obj.exec())
    return EmptyResponse()
class Calls: #Async
    #move_servo = rospy.ServiceProxy("servos_service", Servos)
    def __init__(self, name,execs, static = True):
        if Execute.debug:
            rospy.logwarn(f"Initialising a call {name}, static:{static}")
        self.executables = []
        self.args = []
        self.parsers = []
        self.static = static
        self.name = name   
        if Execute.debug:
            rospy.loginfo(f"Parsing through execs...\n{execs}")
        if static:
            for sub_dict in execs:
                exec_name = list(sub_dict.keys())[0]
                if Execute.debug:
                    rospy.loginfo(f"Parsing {exec_name}")
                self.executables.append(Execute.exec_dict[exec_name]())
                if Execute.debug:
                    rospy.loginfo(f"Appended executable {self.executables[-1]}")
                self.parsers.append(Execute.parsers_dict[exec_name])
                if Execute.debug:
                    rospy.loginfo(f"Appended parser {self.parsers[-1]}")
                pargs = {}
                #print (sub_dict[exec_name])
                for sub_dict_args in sub_dict[exec_name]:
                    pargs = {**pargs, **sub_dict_args}
                self.args.append(pargs)
                if Execute.debug:
                    rospy.loginfo(f"Args = {self.args}")
            rospy.Service(f"{self.name}", Empty, partial(reprExec,obj=self))
        else:
            for exec in execs:
                if Execute.debug:
                    rospy.loginfo(f"Parsing {exec}")
                args = []
                self.executables.append(Execute.exec_dict[exec]())
                if Execute.debug:
                    rospy.loginfo(f"Appended executable {self.executables[-1]}")
                self.parsers.append(Execute.parsers_dict[exec])
                if Execute.debug:
                    rospy.loginfo(f"Appended parser {self.parsers[-1]}")
    def __str__(self):
        return f"Call {self.name}:\n###{self.executables}\n###{self.parsers}\n###{self.args}\n"
    def __repr__(self):
        return f"Call {self.name}:\n###{self.executables}\n###{self.parsers}\n###{self.args}\n"
    async def exec(self,args=None):
        "If the call is dynamic, you should pass args to exec() method! Use dict for kwargs of the service"
        if Execute.debug:
            rospy.loginfo(f"static:{self.static}")
        if self.static:
            proc = asyncio.create_task(self.executeStatic())
        else:
            proc = asyncio.create_task(self.executeDynamic(args))
        return await proc
    async def executeStatic(self):
        resps = []
        async def subExec():
            for exec, args, parser in zip(self.executables, self.args, self.parsers):
                _corout = exec(parser(args))
                if Execute.debug:
                    if Execute.debug:
                        rospy.loginfo(f"Executing static {exec}, parser = {parser}, args = {args}")
                    if Execute.debug:
                        rospy.loginfo(f"Couroutine = {_corout}")
                resps.append(await _corout)
                await asyncio.sleep(0.08)
            if Execute.debug:
                rospy.loginfo(f"Executed static {self.name}, responces = {resps}")
            if 1 in resps:
                return "fail"
            elif not 0 in resps:
                return str(resps[-1])
            else:
                return "done"
        if Execute.debug:
            return await subExec()
        else:
            try:
                return await subExec()
            except:
                rospy.logerr(f"Call {self.name} unavailable!")
                return "fail"
    async def executeDynamic(self,args):
        sub_calls = []
        async def subExec():
            for tup in zip(self.executables, self.parsers):
                exec, parser = tup
                _corout = exec(parser(args))
                sub_calls.append(asyncio.create_task(_corout))
            resps = await asyncio.gather(*sub_calls)
            if Execute.debug:
                rospy.loginfo(f"Executing dynamic {self.name}, responces = {resps}")
            if 1 in resps:
                return "fail"
            elif not 0 in resps:
                return str(resps[-1])
            else:
                return "done"
        if Execute.debug:
            return await subExec()
        else:
            try:
                return await subExec()
            except:
                rospy.logerr(f"Call {self.name} unavailable!")
                return "fail"
    @staticmethod
    def parseServos(args):
        parsed = ServosRequest()
        if Execute.debug:
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
        if Execute.debug:
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
        resp = proxy(args).resp
        if resp < 30:
            return "yellow"
        elif resp < 50:
            return "purple"
        else:
            return "red"
    @staticmethod
    def getOhmExec():
        return Calls.ohmsExec
    ###############################
    @staticmethod
    def getAdjExec():
        return Calls.adjExec
    @staticmethod
    async def adjExec(args):
        proxy = rospy.ServiceProxy("adjust_pos_service", Empty)
        proxy(args)
        return "done"
    @staticmethod
    def parseAdj(_placeholder):
        return EmptyRequest()
    ################################
    @staticmethod
    def getChangeCostExec():
        return Calls.changeCostExec
    @staticmethod
    async def changeCostExec(args):
        proxy = rospy.ServiceProxy("change_cost_service", ChangeCost)
        proxy(args)
        return "done"
    @staticmethod
    def parseChangeCost(cost):
        return ChangeCostRequest(cost)
    ################################
    @staticmethod
    def getAdjToggleExec():
        return Calls.adjToggleExec
    @staticmethod
    async def adjToggleExec(args):
        proxy = rospy.ServiceProxy("adjust_toggle_service", SetBool)
        resp = proxy(args)
        if resp.success == True:
            return "done"
        else:
            return "fail"
    @staticmethod
    def parseAdjToggle(args):
        parsed = SetBoolRequest(data= args['toggle'])
        return parsed
    ################################
    @staticmethod
    def getPinExec():
        return Calls.pinExec
    @staticmethod
    async def pinExec(args):
        proxy = rospy.ServiceProxy("pin_reader_service", PinReader)
        proxy(args)
        return "done"
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
    try:
        return await Calls.LcdExec(parsed)
    except:
        rospy.logerr("No Lcd found!")
#############
class Execute:
    file = rospy.get_param("~calls_file", "config/calls/calls_dict.yaml")
    debug = rospy.get_param("~calls_debug", 1)
    if debug:
        rospy.logerr("CALLS EXECUTER HAS DEBUG ON!")
    #/Params
    #Globals
    dict = {}
    raw_dict = {}
    exec_dict = {
        "servos_service":  Calls.getServoExec,
        "prediction_service": Calls.getLcdExec,
        "ohm_reader_service": Calls.getOhmExec,
        "adjust_pos_service": Calls.getAdjExec,
        "pin_service": Calls.getPinExec,
        "adjust_toggle_service": Calls.getAdjToggleExec,
        "change_cost_service": Calls.getChangeCostExec
    }
    parsers_dict = {
        "servos_service": Calls.parseServos,
        "prediction_service": Calls.parseLcd,
        "ohm_reader_service": Calls.parseOhm,
        "adjust_pos_service": Calls.parseAdj,
        "pin_service": Calls.parsePin,
        "adjust_toggle_service": Calls.parseAdjToggle,
        "change_cost_service":Calls.parseChangeCost
    }
    @classmethod
    def read(cls):
        if Execute.debug:
            rospy.loginfo("Reading file...")
        with open(cls.file, "r") as stream:
            try:
                cls.raw_dict = (yaml.safe_load(stream))
                if Execute.debug:
                    rospy.loginfo(f"Got dict!")
            except yaml.YAMLError as exc:
                if Execute.debug:
                    rospy.logerr(f"Loading failed ({exc})")
    @classmethod
    def parse(cls):
        #if Execute.debug:
        #    rospy.loginfo(f"Raw dict = {cls.raw_dict}")
        static = cls.raw_dict["Static"]
        for call_dict in static:
            call_name = list(call_dict.keys())[0]
            if Execute.debug:
                rospy.loginfo(f"Parsing {call_name}")
            try:
                cls.dict[call_name] = Calls(call_name, call_dict[call_name])
            except:
                if Execute.debug:
                    rospy.logerr(f"Syntax error for static {call_name}! (No available services or duplicate call name)")
        dynamic = cls.raw_dict["Dynamic"]
        for call_dict in dynamic:
            call_name = list(call_dict.keys())[0]
            if Execute.debug:
                rospy.logwarn(f"Parsing {call_name}")
            try:
                cls.dict[call_name] = Calls(call_name, call_dict[call_name], static=False)
            except:
                if Execute.debug:
                    rospy.logerr(f"Syntax error for dynamic {call_name}! (No available services or duplicate call name)")


from ebobot.srv import SetMoveTarget, SetMoveTargetRequest
class Move:
    proxy_name = rospy.get_param("/global/move_service_name", "set_move_service")
    use_actionlib = rospy.get_param("/global/use_actionlib", 1)
    def __init__(self,cb):
        "Init a client with a callback function!"
        self.cb = cb
        if Move.use_actionlib:
            self.client = actionlib.SimpleActionClient('move', MoveAction)
        else:
            #print("EPICEPIC")
            self.client = ProxyClient()
        if Execute.debug:
            rospy.logwarn(f"Waiting for server 'move'...")
        self.client.wait_for_server()
    def setTarget(self,target_pos,target_th):
        if Execute.debug:
            rospy.loginfo(f"##Set new goal for {target_pos}, {target_th}")
        self.target_pos = target_pos
        self.target_th = target_th
        goal = MoveGoal()
        goal.x = target_pos[1]
        goal.y = target_pos[0]
        goal.theta = target_th
        self.client.send_goal(goal, feedback_cb=self.cb)
    def fetchResult(self):
        return self.checkResult()
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
##############################################
class ProxyClient:
    proxy = rospy.ServiceProxy(Move.proxy_name, SetMoveTarget, True)
    rospy.wait_for_service(Move.proxy_name)
    def __init__(self):
        self.state = "init"
        #self.proxy = ProxyClient.proxy
        self._done = True
    def wait_for_server(self):
        rospy.logwarn("CALLS EXEC: USING SERVICES INSTEAD OF ACTIONLIB!!")
    def get_state(self):
        return self.state
    def send_goal(self,goal,*, feedback_cb):
        req = SetMoveTargetRequest(x = goal.x, y = goal.y, theta = goal.theta)
        self._done = False
        resp = self.proxy(req)
        self._done = True
        #self.state = resp.status
        feedback_cb(MoveFeedback(self.state))
        rospy.logwarn(f"EXECUTER: Move done status = {self.state}")
        if not resp.preempted:
            if resp.status == "done":
                self.state = GoalStatus.SUCCEEDED
            elif resp.status == "fail":
                self.state = GoalStatus.ABORTED
            else:
                self.state = GoalStatus.ACTIVE
        else:
            self.state =  GoalStatus.ACTIVE
    def wait_for_result(self):
        while not rospy.is_shutdown() and not self._done():
            rospy.sleep(0.1)
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



