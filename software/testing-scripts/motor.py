import serial
import serial.tools.list_ports
import time
import struct
import threading

#Added by Nick
import pygame
import os

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
                s.write('p')
                device_id = s.readline()
                print(device_id)
                if device_id.strip() == self.id_str.strip():
                    print("## Matched ID ##")
                    s.write('r')
                    self.serial = s
                    return "test"
            except (OSError, serial.SerialException):
                pass
        print ("##! Failed to find match !##")

    def write_packet(self):
        pass
    
#    def debug_read(self):
#        if (serial.inWaiting > 0):
#            raise "Recieved something from the BB"
#            return serial.read()
#        else:
#            return ""
    
class MotorSerial(SerialBoard):
    def __init__(self):   
        self.baud = 9600
        self.timeout = 3
        self.id_str = "DRIVE"
        self.serial = None;
        self.packet_struct = "cccccccc"
        self.left = 0;
        self.right = 0;
        self.pitch = 0;
        self.roll = 0;
        self.yaw = 0;
        self.debugStr = "" #Added by Nick
        
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
        return self.serial.write(packet)
        
class MotorController(object):
    
    def __init__(self):
        self.serial = MotorSerial();
        if(not self.serial.find_port()):
            raise "Motor controler could not be started; Serial connection failed."
        self.left = 0
        self.right = 0
        self.cur_right = 0
        self.cur_left = 0
        self.step = 1
        self.cycle_size = .02
        self.debugStr = "Debug String: " #Added by Nick
    
    
    def change_speed(self, left, right):
        if 0 > left > 255:
            raise str(left)+"(left speed) not in range [0, 255]"
        
        if 0 > right > 255:
            raise str(right)+"(right speed) not in range [0, 255]"
        self.left = left
        self.right = right
	self.serial.write_packet(self.left, self.right, 0, 0, 0)
#        self.debugStr += str(self.serial.write_packet(self.left, self.right, 0, 0, 0))
#        self.debugStr += str(self.left)
#        self.debugStr += "7"
        print(self.debugStr)  #Added by Nick
    
    #
    # Loop to be constantly executed to keep sending packets and to perform
    # movment smoothing
    #
    def loop(self):
        # Perform the smoothing by adjusting the speed we move by the step
        if not self.left == self.cur_left:
            if self.left > self.cur_left:
                self.cur_left += self.step
            else:
                self.cur_left -= self.step
        
#        self.serial.write_packet(self.cur_left, self.cur_right, 0, 0, 0)
        time.sleep(self.cycle_size)
    

if __name__ == '__main__':
  pygame.init()
  pygame.joystick.init()
  MyJoystick = pygame.joystick.Joystick(0)
  MyJoystick.init()
  
  motor = MotorController()
  loop = threading.Thread(target=motor.loop)
  loop.start()
 
  while(1):
    pygame.event.get()
    
    left = int((MyJoystick.get_axis(1)* -127) + 127)
    right = int((MyJoystick.get_axis(3)* -127) + 127)
    os.system('clear')
    print "Left Value: " + str(left)
    print "Right Value: " + str(right)
    motor.change_speed(left, right)
    time.sleep(.005)
 
 
