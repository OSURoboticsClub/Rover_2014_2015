#!/usr/bin/env python2
import rospy
from std_msgs.msg import String

import threading

class StateMachine(object):

    def __init__(self):
        self.full_spd = 215
        self.full_back = 40
        self.zero = 127

        self.left_s = self.zero
        self.right_s = self.zero
        
                
        rospy.init_node('motor_state', anonymous=True)

        self.rate = rospy.Rate(10) #10hz

        self.motor_raw = rospy.Publisher('/motor/motor_raw', String, queue_size=10)
        rospy.Subscriber("/motor/state_change", String, self.state_change)
        
        self.states = {
            "forward": self.forward,
            "backward": self.backward,
            "left": self.left,
            "right": self.right,
            "stop": self.stop
        }

    def forward(self):
        self.left_s = self.full_spd
        self.right_s = self.full_spd

    def backward(self):
        self.left_s = self.full_back
        self.right_s = self.full_back

    def left(self): 
        self.left_s = int((self.full_back-127)/1.5)+127
        self.right_s = int((self.full_spd-127)/1.5)+127

    def right(self):
        self.left_s = int((self.full_spd-127)/1.5)+127
        self.right_s = int((self.full_back-127)/1.5)+127

    def stop(self):
        self.left_s = self.zero
        self.right_s = self.zero
    
    #
    # repeat the speed message so its not missed
    #
    def repeat(self):
        while True:
            self.motor_raw.publish("l%d r%d" % (self.left_s, self.right_s))
            self.rate.sleep()

    #
    # change the state when we recieve the right message
    #
    def state_change(self, data):
        state = self.states.get(data.data, "stop")
        state()


if __name__ == "__main__":
    motor_state = StateMachine()
    pub = threading.Thread(target=motor_state.repeat);
    pub.start()
    rospy.spin()
