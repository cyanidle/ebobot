import roslib
roslib.load_manifest('ebobot')
import rospy
import actionlib
import yaml
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
from ebobot.srv import ServosRequest, ServosResponse
#from ebobot.srv import Catcher
#




def executer_dict():
    return Execute.dict
class Call:
    def __init__(self,name,exec) -> None:

        pass
    pass
class Execute:
    file = rospy.get_param("/task_manager/calls_file", "/config/calls/calls_dict.yaml")
    #/Params
    #Globals
    dict = {}
    execs_dict = {
        #"call": Task.Microtasks.Calls,
        
        }
    ##################### Manager
    # def __init__(self):
    #     sesdasd
    @classmethod
    def read(cls):
        with open(cls.file, "r") as stream:
            try:
                cls.dict = (yaml.safe_load(stream))
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")

    @classmethod
    def parse(cls):
        #interrupts = cls.dict["interrupts"]
        #tasks = cls.dict["tasks"]
        for call_name in cls.dict["interrupts"]:
            
    @staticmethod
    def exec(action):
        action()
    @staticmethod
    def parse():
        pass







Execute.parse()