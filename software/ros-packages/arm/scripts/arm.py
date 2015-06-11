#!/usr/bin/env python2
import serial
import serial.tools.list_ports
import time
import struct
import threading
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
                s.write('p')
                device_id = s.readline()
                print device_id
                if device_id.strip() == self.id_str.strip():
                    s.write("r")
                    self.serial = s
                    print "# Found the correct board #"
                    return "test"
            except (OSError, serial.SerialException):
                pass
        print "##! Cannot find serial port... !##"
    
    def write_packet(self):
        pass
    
    
class ArmSerial(SerialBoard):
    def __init__(self):   
        self.baud = 9600
        self.timeout = 3
        self.id_str = "ARM"
        self.serial = None;
        self.packet_struct = "cccccccc"
        self.x = 0
        self.y = 0
        self.z = 0
        self.command = 0
        
    def checksum(self):
        return 0x76
        
    def write_packet(self, command=None, x=None, y=None, z=None):
        if x == None:
            x = self.x
        else:
            self.x = x
        
        if y == None:
            y = self.y
        else:
            self.y = y
            
        if z == None:
            z = self.z
        else:
            self.z = z
            
        
        if command == None:
            command = self.command
        else:
            self.command = command
        print self.command
        packet = struct.pack(self.packet_struct, chr(0xff), chr(command), chr(x), chr(y), chr(z), chr(0), chr(self.checksum()), chr(0xff))
        self.serial.write(packet)
        self.serial.read(1) 
        time.sleep(15)
        
class ArmController(object):
    
    def __init__(self):
        self.serial = ArmSerial();
        if(not self.serial.find_port()):
            raise Exception("Arm controler could not be started; Serial connection failed.")
        self.cycle_size = 1
        self.x = 0
        self.y = 0
        self.z = 0
        self.command = 0x04
        self.need_move = True
        self.commands = rospy.Subscriber("/arm/commands", String, self.parse_packet)
        self.state = rospy.Publisher("/arm/state", String, queue_size=10)
    
    def move(self, x, y, z, grip):
        if 0 > x > 255:
            raise Exception("x not in range [0, 255]")
        
        if 0 > y > 255:
            raise Exception("y not in range [0, 255]")
        
        if 0 > z > 255:
            raise Exception("z not in range [0, 255]")
        #if grip > 1:
        #    raise Exception("grip must be 1 or 0")
        self.x = x
        self.y = y
        self.z = z
        self.command = grip
        print self.command
        self.need_move = True
    
    #parses the packet and calls move
    #packets structure = "x,y,z,grip"  where 0 < (x == y == z) < 255 and grip = 0|1
    def parse_packet(self, data):
        x,y,z,grip = data.data.split(",")
        self.move(int(x), int(y), int(z), int(grip))
    #
    # Loop to be constantly executed to keep sending packets and to perform
    # movment smoothing
    #
    def loop(self):
        while True:
            if self.need_move:
                self.serial.write_packet(self.command, self.x, self.y, self.z)
                self.need_move = False
                if self.command == 4:
                    self.command = 0
            self.state.publish(String(str(1) if self.need_move else str(0)))
            time.sleep(self.cycle_size)
    

if __name__ == '__main__':
    rospy.init_node("arm_controller")
    arm = ArmController()
    loop = threading.Thread(target=arm.loop)
    loop.start()
    rospy.spin() 
