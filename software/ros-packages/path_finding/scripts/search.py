#!/usr/bin/env python2
import rospy
import time
from std_msgs.msg import String 
if __name__ == "__main__":
    pub = rospy.Publisher('/motor/commands/path_finding', String, queue_size=10)
    rospy.init_node('search')
    pub.publish("f100")
    pub.publish("r270")
    pub.publish("f1000")
    time.sleep(20)
    pub.publish("flush");
    i = 0
    while not rospy.is_shutdown():
        print i
        pub.publish("f"+str(i*100))
        time.sleep(i*2)
        pub.publish("flush")
        pub.publish("r270")
        time.sleep(4)
        i += 1;
