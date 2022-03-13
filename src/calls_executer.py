import roslib
roslib.load_manifest('ebobot')
import rospy
import actionlib
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
from ebobot.srv import ServosRequest, ServosResponse
#from ebobot.srv import Catcher
#
class Execute:
    pass