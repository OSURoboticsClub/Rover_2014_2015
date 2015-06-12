#!/usr/bin/python2
import cv2
import rospy
import std_msgs
import math

class FindStart():
    def __init__(self): 
        self.motor = rospy.Publisher("/motor/commands", std_msgs.msg.String)
        self.focal = 16.0
        self.baseline = None
        #angle per pixel
        self.app = .09375
        #the height of the camera off the ground
        self.cam_height = .25
        #the width and height of the square
        self.square_back = .2032
        self.square = 0.1778
        self.dim = (5, 3)
        self.dim_small = (4, 3)
        self.started = 1
        self.searching = False
        self.search_turn = False

    def points(self, data):
        self.focal = data.f/6000
        self.baseline = data.T
    
    def checker_board(self, img):
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(img, self.dim, None)
        ret_back, corners_back = cv2.findChessboardCorners(img, self.dim_small, None)
        print "Checkerboard:",
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if ret:
            print "forward"
            #cv2.drawChessboardCorners(gray, self.dim, corners, ret)
            height = self.get_height(img, corners)
            center = self.get_center(corners)
            distance = self.get_distance(gray, height)
            angle = self.get_angle(center)
            print height, center, distance, angle

        if ret_back:
            print "back"
            height = -self.get_height(img, corners_back, "small")
            center = self.get_center(corners_back)
            distance = self.get_distance(gray, height, "back")
            angle = self.get_angle(center)
            print height, center, distance, angle

        if ret or ret_back:
            self.motor.publish("flush")
	
        if ret:
            threshold = math.asin(.5/distance)
                distance -= 10
                if distance > 20:
                    distance = int(distance*(3.0/4))
                else:
                    return False
                if -threshold < angle < threshold:
                    print "Moving forward", distance, "angle was", angle
                    self.motor.publish("f%d" % (int(distance)))
                else:
                    if angle < 0:
                        angle = 360+angle
                    print "Rotating", angle, "and moving forward", distance
                    self.motor.publish("r%df%d" % (int(angle), int(distance)))
                print distance, angle, threshold
                if distance < 20 and (angle < max(threshold, 8) or angle > min(360-threshold, 352)):
                    print "Final form entered"
                    self.searching = False
                    self.search_turn = False
                    self.change_state.publish("Found Base Station Final")
                else:
                    self.searching = True
                    self.searh_turn = False
                while rospy.wait_for_message("/motor_status", std_msgs.msg.String).data == "busy":
                    pass
        
        if ret_back:
            distance *= 2
            self.motor.publish("r45f%dr45%d" % (int(distance), int(distance))) 
            while rospy.wait_for_message("/motor_status", std_msgs.msg.String).data == "busy":
                pass
        elif self.searching:
            if not self.search_turn:
                self.motor.publish("r315")
                self.search_turn = True
            else:
                self.motor.publish("r10")
        return True

    def get_height(self, img,  grid, size="large"):
        if size == "large":
            dim = self.dim
        else:
            dim = self.dim_small
            
        start = grid[0][0][1]
        end = grid[-dim[0]][0][1]
        cv2.line(img, tuple(grid[0][0]), tuple(grid[-dim[0]][0]), (255, 0, 0))
        return end-start

    def get_distance(self, img, height, dim="large"):
        self.focal = (0.5 * img.shape[1] / math.tan(0.5 * 65 * math.pi / 180))*(4.2/1000.0);
        if self.focal is not None:
            if dim == "large":
                return (2.0 / 2.7) *(self.focal*(self.square*(self.dim[0]-2))*img.shape[1])/(height*self.cam_height)
            else:
                return (self.focal*(self.square_back*(self.dim_small[0]-2))*img.shape[1])/(height*self.cam_height)	
        return -1

    def get_center(self, grid, side="front"):
        if side == "front":
            return grid[-self.dim[0]/2][0][0]
        else:
            return grid[-self.dim_back[0]/2][0][0]

    def get_angle(self, center):
        return (center*self.app)-30

#TODO: Any cleanup??

if __name__ == '__main__':
    try:
        global interrupt
        interrupt = False
        def handler(sig, f):
            global interrupt
            interrupt = True
            sys.exit()
        rospy.init_node("Home_search", anonymous=True)
        #handle lethal signals in order to stop the motors if the script quits
        signal.signal(signal.SIGINT, handler)
        start = FindStart()
        while True:
            try:
                VideoCapture cap(0);
                break
            except:
                pass
        while not interrupt:
            start.image()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    self.change_state = rospy.Publisher("/motor/commands", std_msgs.msg.String)
