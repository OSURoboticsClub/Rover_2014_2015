#!/usr/bin/env python
import itertools
import time

import rospy
from std_msgs.msg import String

class QueueProc(object):

    def __init__(self):
    	rospy.init_node('motor_queue_proc', anonymous=True)
	
	self.state_change = rospy.Publisher('/motor/state_change', String, queue_size=10)
        rospy.Subscriber("/motor/commands", String, self.parse_string)

        self.queue = []
        self.timer_class = TimeTimer
    
    #
    # Parses published strings and loads the next item into the queue
    #
    def parse_string(self, data):
	commands = ["".join(x) for _, x in itertools.groupby(data.data, key=str.isdigit)]
	queue_start = len(self.queue)
	i = 0
	while i < len(commands):
	    action = commands[i]
	    val = commands[i+1]
	    if action == "f":
		self.queue.append(["forward", int(val)])
	    elif action == "b":
		self.queue.append(["backward", int(val)])
	    elif action == "r":
		rounded = int(val)%360
		if rounded > 180:
		    rounded = 360-rounded
		    self.queue.append(["left", rounded])
		else:
		    self.queue.append(["right", rounded])
	    elif action == "flush":
		self.queue = []
	    i += 2
	if queue_start == 0:
	    self.timer_expire()
    
    #
    # changes the state and sets a timer based on the next item in the queue
    #
    def timer_expire(self):
	if len(self.queue) == 0:
	    self.state_change.publish("stop")
	    return
	nxt = self.queue.pop(0)
        if (nxt[0] == "left" or nxt[0] == "right"):
            self.state_change.publish("stop")
            time.sleep(2)
	self.state_change.publish(nxt[0])
	tmer = self.timer_class(nxt[1], self.timer_expire, (nxt[0] == "left" or nxt[0] == "right"))
        tmer.start()

#
# General timer class, does nothing
#
class GeneralTimer():
    def __init__(self, distance, callback, is_angle):
    	self.callback = callback

    def start(self):
    	self.callback()

    def get_time(self):
        return 0


#
# A time based timer
#
class TimeTimer(GeneralTimer):

    def __init__(self, distance, callback, is_angle):
    	self.callback = callback
    	#distance in m*10
    	self.distance = distance
    	self.is_angle = is_angle
    	#meters per second
    	self.mps = 1
    	#amount of angles turned per second
    	self.aps = 40

    def start(self):
	rospy.Timer(rospy.Duration(self.get_time()), self.timer_callback, True)


    def timer_callback(self, tEvent):
	self.callback()

    def get_time(self):
        if not self.is_angle:
            return float(self.distance)/(self.mps*10)
        else:
            return self.distance/self.aps 


if __name__ == "__main__":
    proc = QueueProc()
    rospy.spin()
