#!/usr/bin/env python2
import rospy
import time
from std_msgs.msg import String

class ArmStates(object):
    def __init__(self):
        #a double layer array to store the bucket positions
        self.buckets = [
            [35, 200, 180],        
            [120, 200, 180]
        ]
        self.objects = 0
        self.home = [100, 150, 0, 0] 
        self.grab_z = 255
        self.working = False
        self.get_object = rospy.Subscriber("/arm/pickup", String, self.pickup)
        self.arm = rospy.Publisher("/arm/commands", String, queue_size=10)
        self.arm_state = rospy.Subscriber("/arm/state", String, self.state_change)
        self.toggle_handler = rospy.Subscriber("/arm/toggle", String, self.toggle)



    def store(self):
        print "Storing", self.objects, "in", len(self.buckets)
        if len(self.buckets) > self.objects:
            print "Stored"
            x,y,z = self.buckets[self.objects]
            self.move([x, y, 0, 1])
            self.move([x, y, z, 1])
            self.move([x+30, y, z, 0])
            self.move([x+30, y, 0, 0])
            self.move(self.home)
            self.objects += 1
        else:
            print "Out of Buckets"

    def move(self, position):
        print "Moving"
        while self.working:
            pass
        self.arm.publish(String("%d,%d,%d,%d" % tuple([int(x) for x in position])))
        self.working = True

    def state_change(self, data):
        if data.data == "0":
            self.working = False

    def grab(self, position):
        x,y = position
        z = self.grab_z
        self.move([x, y, 0, 0])
        self.move([x, y, z, 0])
        self.move([int(x)-30, y, z, 1])
        self.move([int(x)-30, y, 0, 1])

    def pickup(self, data):
        position = data.data.split(",")
        if not len(position) == 2:
            print "Invalid Position, needs an x and a y"
        else:
            self.grab(position)
            self.store()
            
    def toggle(self, lower):
        #All of the following values need calibrating
        grip_hard_x = 100 
        grip_hard_y = 150
        grip_hard_z = 240
        
        store_hard_x = grip_hard_x
        store_hard_y = 200 #Calibrate!
        store_hard_z = 0
        
        if(int(lower) > 0):  #Lower case
            self.move([grip_hard_x, grip_hard_y, 0,           4]) #Home the arm
            self.move([grip_hard_x, grip_hard_y, 0,           0]) 
            self.move([grip_hard_x, grip_hard_y, grip_hard_z, 0])
        else:                #Raising case
            self.move([grip_hard_x,  grip_hard_y,  0,            0])
            self.move([store_hard_x, store_hard_y, store_hard_z, 0])

if __name__ == "__main__":
    rospy.init_node("arm_states")
    arm = ArmStates()
    rospy.spin()
