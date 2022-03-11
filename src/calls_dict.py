import roslib
roslib.load_manifest('ebobot')
import rospy
import actionlib
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback, Catcher, CatcherRequest, CatcherResponse#, MoveGoal
#from ebobot.srv import Catcher
#
