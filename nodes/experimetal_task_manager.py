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


class Status:
    list = []
    def __init__(self, parent):
        self._status = "init"
        self.parent = parent
    def update(self):
        self._status = self.parent.updateStatus()
    def get(self):
        return self._status
    def set(self, text):
        self._status = text
class Template(ABC):
    list = []
    class_name = "<NameNotSet>"
    @abstractmethod
    def __init__(self,parent,name,args):
        self.parent = parent
        self.num = type(self).counter
        type(self).counter += 1
        self.name = name
        self.status = Status(self)
        type(self).list.append(self)
    @abstractmethod
    def exec(self):
        self.status.set("executing")
    def updateStatus(self):
        return self.status.get()  
    def __repr__(self):
        return f"<{type(self).class_name} {self.num}: {self.name}>"

class Task(Template):
    class_name = "Task"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
    def exec(self):
        super().exec()
        for micro in self.micros_list:
            micro.exec()
            while Interrupts.todo_list:
                inter = Interrupts.todo_list.pop()
                inter.exec()
class Timer(Template):
    class_name = "Timer"
    def __init__(self, parent, name, args):
        super().__init__(parent, name, args)
        

constructors_dict = {  #syntax for route.yaml
        "call":Call,
        "log": Log,
        "move": Move,
        "condition":Condition,
        "group": Group,
        "interrupt": Interrupt.forceCallParse,
        "skip": Skip,
        "score": Prediction,
        "dynamic_call": Dyn_call,
        "sleep": Sleep,
        "goto": Jump
        } 
