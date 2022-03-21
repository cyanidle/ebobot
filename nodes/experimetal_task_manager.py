#!/usr/bin/env python3
import roslib
roslib.load_manifest('ebobot')
import rospy
import yaml
from threading import Thread
import asyncio
rospy.init_node("task_manager")
#
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponce
#
from markers import pubMarker
from calls_executer import executer_dict, showPrediction
from calls_executer import Move as move_client_constructor
#
from ebobot.msg import MoveAction, MoveResult, MoveFeedback#, MoveGoal
#
from abc import ABC, abstractmethod

class Template(ABC):
    @abstractmethod
    def __init__(self,parent,name,args):
        self.parent = parent
        self.num = type(self).counter
        type(self).counter += 1
        self.name = name
        self.status = "init"
    @abstractmethod
    def getStatus(self):
        return self.status



class Status:   ### EACH OBJECT WHICH IS ADDED TO STATUS SERVER SHOULD HAVE A STATUS AND UPDATE STATUS METHOD,
    calls = []  ### STATUS RETURNS THE DESIRED VALUE, WHILE UPDATE JUST GETS CALLED EACH STATUS SERVER UPDATE CYCLE
    moves = []
    timers = []
    interrupts = []
    
    class Timer(Template):     
        counter = 0
        def __init__(self,main=False) -> None:
            self.main = main
            self.num = type(self).counter
            type(self).counter += 1
            self.time = 0
            self.ros_time = rospy.Time.now()
            Status.add(self)
        def status(self):
            return self.time
        def updateStatus(self):
            self.time = (rospy.Time.now() - self.ros_time).to_sec()
        def __str__(self):
            return f"Timer {self.num} (Main = {self.main})"
        def __repr__(self):
            return f"Timer {self.num} (Main = {self.main})"
   