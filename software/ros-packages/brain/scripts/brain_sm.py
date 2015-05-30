#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import simplejson

class StateMachine(object):
    def __init__(self, state_json_file):
        self.states = {}
        self.forward = rospy.Publisher('/motor/commands', String, queue_size=10)  
        self.brain_state = rospy.Subscriber('/brain/state_change', String, self.state_change_request)
        #open the json file
        with open(state_json_file) as data_file:    
            data = simplejson.loads(data_file.read())
        
        forwards = data["forwards"]
        #loop through and create each state
        for state in data["states"]:
            print "Creating State:", state["name"]
            st = State(state["name"])
            #set motor forwards
            for f in state["forward"]:
                if f in forwards:
                    st.set_motor_forward(f)
                else:
                    print "Invalid Forward",f
            self.states[state["machine_name"]] = st

        for tran in data["transitions"]: 
            for frm in tran["from"]:
                try:
                    self.states[frm].add_transition(self.states[tran["to"]], tran["name"])
                except KeyError:
                    print "Invalid Transition:",frm, "->",tran["to"]

        self.cur_state = None
        self.set_state(self.states[data["init"]])

    #this is a callback function that i0s called when a motor command is recieved 
    #the text from that message is then forwarded to the motors
    def forward_data(self, data):
        self.forward.publish(data)
        
    #this moves over to a new state and deactivates the old one
    def set_state(self, state):
        if not self.cur_state == None:
            self.cur_state.deactivate()
        self.cur_state = state
        print self.cur_state
        state.activate(self.forward_data)

    def state_change_request(self, data):
        st = self.cur_state.get_transition(data.data)
        if not st:
            print "Invalid state change request:", data.data
        else:
            self.set_state(st)

class State(object):
    def __init__(self, name):
        #our transitions to other states
        self.transitions = {}
        #a list of topics to forward to the motor controller
        self.motor_forwards = []
        #a list of all subscribed topics
        self.subs = []
        #the state name
        self.state_name = name


    def add_transition(self, state, trans_string):
        self.transitions[trans_string] = state

    def get_transition(self, trans_string):
        return self.transitions.get(trans_string, False)

    def set_motor_forward(self, topic):
        self.motor_forwards.append(topic)

    def activate(self, callback):
        for topic in self.motor_forwards:
            rospy.Subscriber(topic, String, callback)

    def deactivate(self):
        for sub in self.subs:
            sub.unregister()

    def __str__(self):
        return "State: "+self.state_name

if __name__ == "__main__":
    rospy.init_node('brain_statemachine')
    sm = StateMachine("statemachine.json")
    rospy.spin()
