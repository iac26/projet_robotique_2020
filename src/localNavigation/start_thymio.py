import sys
import time
import serial
import math
from threading import Timer

sys.path.append("localNavigation")
from Thymio import Thymio

sys.path.append("vision")
import vision

value_proximity=[0,0,0,0,0,0,0]
value_acceleration=[0,0]
value_speed=[0,0]
actual_position=[0,0]
actual_angle=0
error_sum=0



class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False




def connexion_thymio():
    global th
    th = Thymio.serial(port="COM7", refreshing_rate=0.1)
    time.sleep(3) # To make sure the Thymio has had time to connect
    print("Thymio is connected :)")


def measure_sensor():
    global value_proximity
    global value_acceleration
    global value_speed

    value_proximity=th['prox.horizontal']
    value_acceleration=th['acc']
    value_speed=[th['motor.left.speed'],th['motor.right.speed']]
    for i in range(2):
        if value_speed[i]>600:
            value_speed[i]=value_speed[i]-2**16
    for i in range(3):
        if value_acceleration[i]>600:
            value_acceleration[i]=value_acceleration[i]-2**16
    
    #return value_proximity,value_acceleration,value_speed

def get_sensor_value():
    return value_proximity,value_acceleration,value_speed



def get_position(cap):
    global actual_angle
    global actual_position
    #get one frame
    ret, frame = cap.read()
    scale = vision.detect_scale(frame)
    robot_pos, ret = vision.detect_robot(frame, scale)
    #print(robot_pos)
    #print("robot position:",robot_pos[0])
    #print("robot angle:",robot_pos[1])
    if robot_pos[2]==True:
        #print("robot position:",robot_pos[0])
        #print("robot angle:",robot_pos)
        actual_position= [robot_pos[0][0],robot_pos[0][1]] 

        actual_angle=robot_pos[1]
        return actual_position,actual_angle
    else: 
        return actual_position,actual_angle
    

def compute_angle_goal (actual_position,goal,actual_angle):
    return math.atan((goal[1]-actual_position[1])/(goal[0]-actual_position[0]))-actual_angle




def follow_the_way_to_dream(actual_position,goal,actual_angle):
    """
    base_speed, kp,ki à tunner
    """
    global error_sum 
    base_speed = 80
    kp = 100
    ki = 3.5
    saturation = 500
    error_sat=500/ki
    error = compute_angle_goal(actual_position,goal,actual_angle)
    error_sum = error_sum + error

    if error_sum>error_sat:
        error_sum=error_sat
    if error_sum<-error_sat:
        error_sum=-error_sat

    speed_l= base_speed + (kp*error+ki*error_sum)
    speed_r= base_speed - (kp*error+ki*error_sum)
 

    if speed_l>saturation:
        speed_l=saturation
    if speed_l<-saturation:
        speed_l=-saturation
    if speed_r>saturation:
        speed_r=saturation
    if speed_r<-saturation:
        speed_r=-saturation

    print("total error: ",error_sum)    
    print("error: ",error)    
    print("speed left: ",speed_l)    
    print("speed right: ",speed_r) 
    move(int(speed_l),int(speed_r))
    

def move(l_speed=500, r_speed=500, verbose=False):
    """
    Sets the motor speeds of the Thymio 
    param l_speed: left motor speed
    param r_speed: right motor speed
    param verbose: whether to print status messages or not
    """
    # Printing the speeds if requested
    if verbose: print("\t\t Setting speed : ", l_speed, r_speed)
    
    # Changing negative values to the expected ones with the bitwise complement
    l_speed = l_speed if l_speed>=0 else 2**16+l_speed
    r_speed = r_speed if r_speed>=0 else 2**16+r_speed

    # Setting the motor speeds
    th.set_var("motor.left.target", l_speed)
    th.set_var("motor.right.target", r_speed)