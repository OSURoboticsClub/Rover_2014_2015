#!/usr/bin/env python2

import serial
import serial.tools.list_ports
import time
import struct
import threading
import signal
import itertools

#ros packages
import rospy
from std_msgs.msg import String


class SerialBoard(object):
    def __init__(self):   
        self.baud = 9600
        self.timeout = 3
        self.id_str = ""
        self.serial = None;
        self.packet_struct = ""
        
    #
    # Automaticlly find and connect to a port
    #
    def find_port(self):
        for port in list(serial.tools.list_ports.comports()):
            try:
                s = serial.Serial(port[0], self.baud, timeout=self.timeout);
                time.sleep(2)
                s.write('p')
                device_id = s.readline()
                if device_id.strip() == self.id_str.strip():
                    s.write("r")
                    self.serial = s
                    return True
            except (OSError, serial.SerialException) as e:
                pass
        return False
    
    def write_packet(self):
        pass

    def close(self):
        self.serial.close()
    
    
class MotorSerial(SerialBoard):
    def __init__(self):   
        #serial settings
        self.baud = 9600
        self.timeout = 3
        self.id_str = "Motor Controller"
        self.serial = None;
        #serial packet vars
        self.packet_struct = "cccccccc"
        self.left = 0;
        self.right = 0;
        self.pitch = 0;
        self.roll = 0;
        self.yaw = 0;
        
    def write_packet(self, left=0, right=0, pitch=0, roll=0, yaw=0):
        if left == 0:
            left = self.left
        else:
            self.left = left
    
        if right == 0:
            right = self.right
        else:
            self.right = right
            
        if pitch == 0:
            pitch = self.pitch
        else:
            self.pitch = pitch
        
        if roll == 0:
            roll = self.roll
        else:
            self.roll = roll
        
        if yaw == 0:
            yaw = self.yaw
        else:
            self.yaw = yaw
        packet = struct.pack(self.packet_struct, chr(0xff), chr(left), chr(right), chr(pitch), chr(roll), chr(yaw), chr(~(left ^(right/2)) & 0xff), chr(0xff))
        self.serial.write(packet)
        #read the return packet although we dont use it yet
        self.serial.read(15)
        
class MotorController(object):
    
    def __init__(self):
        self.serial = MotorSerial();
        if(not self.serial.find_port()):
            raise Exception("Motor controler could not be started; Serial connection failed.")
        self.left = 0
        self.right = 0
        self.cur_right = 0
        self.cur_left = 0
        self.step = 1
        self.cycle_size = .01
        #ros vars
        self.topic = "/motor/motor_raw"
    
    def change_speed(self, left, right):
        if 0 > left > 255:
            raise Exception(str(left)+"(left speed) not in range [0, 255]")
        
        if 0 > right > 255:
            raise Exception(str(right)+"(right speed) not in range [0, 255]")
        self.left = left
        self.right = right
    
    #
    # Loop to be constantly executed to keep sending packets and to perform
    # movment smoothing
    #
    def loop(self):
        while True:
            # Perform the smoothing by adjusting the speed we move by the step
            if not self.left == self.cur_left:
                if self.left > self.cur_left:
                    self.cur_left += self.step
                else:
                    self.cur_left -= self.step

            if not self.right == self.cur_right:
                if self.right > self.cur_right:
                    self.cur_right += self.step
                else:
                    self.cur_right -= self.step
            
            self.serial.write_packet(int(self.cur_left), int(self.cur_right))
            time.sleep(self.cycle_size)
    
    #
    # Clean close the serial
    #
    def close(self, signal, frame):
        self.serial.close()
    #
    # Handles Ros call backs for when motor commands are published
    #
    def sub_callback(self, data):
        args = ["".join(x) for _, x in itertools.groupby(data.data, key=str.isdigit)]
        i = 0;
        while len(args) > i:
            action = args[i].strip()
            speed = int(args[i+1])
            if action == "r":
                self.change_speed(self.left, speed)
            elif action == "l":
                self.change_speed(speed, self.right)
            else:
                print "huh...", "'"+str(action)+"'"
                return
            i += 2
    
    def start_subscriber(self):
        #init our motor controller node
        rospy.init_node('motor_controller', anonymous=True)
        
        #set up our subscriber
        rospy.Subscriber(self.topic, String, self.sub_callback)

        #loop and wait for data
        rospy.spin()

if __name__ == '__main__': 
    motor = MotorController()

    signal.signal(signal.SIGINT, motor.close)

    packet_send_thread = threading.Thread(target=motor.loop)
    packet_send_thread.start()
    motor.start_subscriber()

