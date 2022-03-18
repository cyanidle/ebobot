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
    lcd_show = rospy.ServiceProxy("lcd_service", Lcd_show)
    def __init__(self, name,execs):
        print(f"Initialising a call {name}")
        self.executables = []
        self.args = []
        self.parsers = []
        self.name = name   
        print(f"Parsing through execs...\n{execs}")
        for exec_name in execs:
            print(f"Parsing {exec_name =}")
            args = []
            self.executables.append(Execute.exec_dict[exec_name]())
            print(f"Appended executable {self.executables[-1]}")
            self.parsers.append(Execute.parsers_dict[exec_name])
            print(f"Appended parser {self.parsers[-1]}")
            for arg in execs[exec_name].items():
                args.append(arg)
            self.args.append(args)
            print(f"Appended args {self.args[-1]}")
        return self.execute
    async def execute(self):
        sub_calls = []
        for exec, args, parser in zip(self.executables, self.args, self.parsers):
            sub_calls.append(asyncio.create_task(exec(parser(args))))
        resp = await asyncio.gather(*sub_calls)
        rospy.loginfo(f"Executing {self.name}, responces = {resp}")
        if 0 in resp:
            return 1
        else:
            return 0
    @staticmethod
    def parseServos(args):
        parsed = ServosRequest()
        parsed.num = args["num"]
        parsed.state = args["state"]
        return parsed
class Static(Calls):
    pass
class Dynamic(Calls):
    pass
##############
def showPrediction(num):
    parsed = Lcd_showRequest()
    parsed.num = num
    return Calls.lcd_show(parsed)
################################       
class Execute:
    file = rospy.get_param("/task_manager/calls_file", "/config/calls/calls_dict.yaml")
    #/Params
    #Globals
    dict = {}
    raw_dict = {}
    exec_dict = {
        "servos_service":  Calls.getServoExec
    }
    parsers_dict = {
        "servos_service": Calls.parseServos
    }
    @classmethod
    def read(cls):
        print("Reading file")
        with open(cls.file, "r") as stream:
            try:
                cls.raw_dict = (yaml.safe_load(stream))
                print(f"Got dict!\n{cls.raw_dict}")
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")
    @classmethod
    def parse(cls):
        for call_name in list(cls.raw_dict.keys()):
            cls.dict[call_name] = Calls(cls.raw_dict[call_name])

class Move:
    def __init__(self,cb):
        "Init a client with a callback function!"
        self.cb = cb
        self.client = actionlib.SimpleActionClient('move', MoveAction)
        print(f"Waiting for server 'move'...")
        self.client.wait_for_server()
    def setTarget(self,target_pos,target_th):
        print(
            f"##Set new goal for {target_pos = }, {target_th = }, with {self.cb = }")
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
            return 0
        elif state == GoalStatus.SUCCEDED:
            return 1
        elif state == GoalStatus.ABORTED:
            return 2
        elif state == GoalStatus.LOST:
            return 3
        else :
            return 4
    def waitResult(self):
        self.client.wait_for_result()


print("Parsing calls...")
Execute.parse()
print("Done parsing calls!")
print(f"{Execute.dict = }")