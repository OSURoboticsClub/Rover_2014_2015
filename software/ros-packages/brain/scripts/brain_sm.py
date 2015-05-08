#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class StateMachine(object):
    def __init__(self):
        pass



class State(object):
    def __init__(self):
        #our transitions to other states
        self.transitions = {}
        #a list of topics to forward to the motor controller
        self.motor_forwards = []
        #a list of all subscribed topics
        self.subs = []

    def add_transition(self, state, trans_string):
        self.transitions[trans_string] = state

    def set_motor_forward(self, topic):
        self.motor_forwards.append(topic)

    def activate(self, callback):
        for topic in self.motor_forwards:
            rospy.Subscriber(topic, String, callback)

    def deactivate(self):
        for sub self.subs:
            sub.unregister()
