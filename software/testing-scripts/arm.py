import serial
import serial.tools.list_ports
import time
import struct
import threading

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
        packet = struct.pack(self.packet_struct, chr(0xff), chr(command), chr(x), chr(y), chr(z), chr(0), chr(self.checksum()), chr(0xff))
        self.serial.write(packet)
        self.serial.read(1)
        
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
    
    
    def move(self, x, y, z, grip):
        if 0 > x > 255:
            raise Exception("x not in range [0, 255]")
        
        if 0 > y > 255:
            raise Exception("y not in range [0, 255]")
        
        if 0 > z > 255:
            raise Exception("z not in range [0, 255]")
#        if grip > 1:
#            raise Exception("grip must be 1 or 0")
        self.x = x
        self.y = y
        self.z = z
#        self.command = grip
    
    #
    # Loop to be constantly executed to keep sending packets and to perform
    # movment smoothing
    #
    def loop(self):
        while True:
            self.serial.write_packet(self.command, self.x, self.y, self.z)
            if( self.command == 0x04 ):
                self.command = 0
#            print("Wrote packet, command = " + str(self.command) )
            time.sleep(self.cycle_size)
    

if __name__ == '__main__':
  arm = ArmController()
  loop = threading.Thread(target=arm.loop)
  loop.start()

  firstLoop = 1
 
  while(1):
    x = int(raw_input("x:"))
    y = int(raw_input("y:"))
    z = int(raw_input("z:"))
    print("\n") #Added to make inputting numbers a little nicer
    try:
        arm.move(x, y, z, 0x04)
    except Exception, e:
        print e
 
