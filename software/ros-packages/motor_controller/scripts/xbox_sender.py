#!/usr/bin/env python2
import time
import struct
import threading
import rospy
from std_msgs.msg import String

#Added by Nick
import pygame
import os
motor_state = rospy.Publisher('/motor/state_change', String, queue_size=10)
def send():
    while(1):
        pygame.event.get() 
        fb = int((MyJoystick.get_axis(1)* -127) + 127)
        lr = int((MyJoystick.get_axis(2)* -127) + 127)
        halt = abs(fb-lr) < 20
        if fb > lr and not halt:
            if fb > 127:
                motor_state.publish(String("forward"))
            else:
                motor_state.publish(String("backward"))
        elif not halt:
            if lr > 127:
                motor_state.publish(String("right"))
            else:
                motor_state.publish(String("left"))
        else:
            motor_state.publish(String("stop"))

        os.system('clear')
        print "forward Value: " + str(fb)
        print "left Value: " + str(lr)
        time.sleep(.005)  #Super-duper hacky




if __name__ == '__main__':
  pygame.init()
  pygame.joystick.init()

  MyJoystick = pygame.joystick.Joystick(0)
  MyJoystick.init()
  
  rospy.init_node('xbox_sender')
  thread = threading.Thread(target=send)
  thread.start()
  rospy.spin()
   
     
 
