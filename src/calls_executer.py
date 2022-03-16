import roslib
roslib.load_manifest('ebobot')
import rospy
import actionlib
import yaml
import asyncio
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback, MoveGoal
from ebobot.srv import Servos, ServosRequest, ServosResponse
from actionlib_msgs.msg import GoalStatus
#from ebobot.srv import Catcher
#
def executer_dict():
    return Execute.dict
def getMoveClient():
    return Move()
class Calls:
    move_servo = rospy.ServiceProxy("servos_service", Servos)
    def __init__(self, name,execs):
        rospy.loginfo(f"Initialising a call {name}")
        self.executables = []
        self.args = []
        self.parsers = []
        self.name = name   
        rospy.loginfo(f"Parsing through execs...\n{execs}")
        for exec_name in execs:
            rospy.loginfo(f"Parsing {exec_name =}")
            args = []
            self.executables.append(Execute.exec_dict[exec_name])
            rospy.loginfo(f"Appended executable {self.executables[-1]}")
            self.parsers.append(Execute.parsers_dict[exec_name])
            rospy.loginfo(f"Appended parser {self.parsers[-1]}")
            for arg in execs[exec_name].items():
                args.append(arg)
            self.args.append(args)
            rospy.loginfo(f"Appended args {self.args[-1]}")
        return self.execute
    def execute(self):
        for exec, args, parser in zip(self.executables, self.args, self.parsers):
            exec(parser(args))
    @staticmethod
    def parseServos(args):
        parsed = ServosRequest()
        parsed.num = args["num"]
        parsed.state = args["state"]
        return parsed
################################       
class Execute:
    file = rospy.get_param("/task_manager/calls_file", "/config/calls/calls_dict.yaml")
    #/Params
    #Globals
    dict = {}
    raw_dict = {}
    exec_dict = {
        "servos_service":  Calls.move_servo
    }
    parsers_dict = {
        "servos_service": Calls.parseServos
    }
    @classmethod
    def read(cls):
        rospy.loginfo("Reading file")
        with open(cls.file, "r") as stream:
            try:
                cls.raw_dict = (yaml.safe_load(stream))
                rospy.loginfo(f"Got dict!\n{cls.raw_dict}")
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")
    @classmethod
    def parse(cls):
        for call_name in cls.raw_dict:
            cls.dict[call_name] = Calls(cls.raw_dict[call_name])

class Move:
    def __init__(self,cb:function):
        "Init a client with a callback function!"
        self.cb = cb
        self.client = actionlib.SimpleActionClient('move', MoveAction)
        rospy.loginfo(f"Waiting for server 'move'...")
        self.client.wait_for_server()
    def setTarget(self,target_pos,target_th):
        rospy.loginfo(
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


rospy.loginfo("Parsing calls...")
Execute.parse()
rospy.loginfo("Done parsing calls!")