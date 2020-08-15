from Ultrasonic_Sensor import *
import math
import time
from calibration import calibration
from motorFunction import pid,pid_wall
from Foward_Kinematics import FWDKIN
from Encoder import *
import board
import busio
import adafruit_bno055
from adafruit_motorkit import MotorKit
import matplotlib.pyplot as plt 
import enum

Left = 1 # for the left sensor
Right = 2 # for the right sensor
kit = MotorKit()

class state(enum.Enum): 
    NONE = 0
    DEFAULT = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    WALL_RIGHT = 4
    WALL_LEFT = 5
    REACH_DESTINATION = 6
    CRASH = 7


def x_y_graph(x, y):
	number_list = list(range(len(x)))
	plt.plot(number_list,x, label = "x_location")
	plt.plot(number_list,y, label = "y_location")
	plt.xlabel('number of loops')
	plt.ylabel('distance')
	plt.show()
	
def x_y_graph2(x, y):
	plt.plot(x,y, label = "distance travel")
	plt.xlabel('x')
	plt.ylabel('y')
	plt.show()
    
def angle_graph(angle):
	number_list = list(range(len(angle)))
	plt.plot(number_list, angle, label = "angle")
	plt.xlabel('number of loops')
	plt.ylabel('angle')
	plt.show()    

def control_speed(left, right):
	kit.motor1.throttle = left
	kit.motor2.throttle = left
	kit.motor3.throttle = right
	kit.motor4.throttle = right
	
def mirror_sensor_angle(angle):
	return 360 - angle

#need to complete later after testing number
def turn_right():
    kit.motor1.throttle = 1
    kit.motor2.throttle = 1
    kit.motor3.throttle = 0.3
    kit.motor4.throttle = 0.3
    return

#need to complete later after testing number
def turn_left():
    kit.motor1.throttle = 0.3
    kit.motor2.throttle = 0.3
    kit.motor3.throttle = 1
    kit.motor4.throttle = 1
    return

#use to read the sensor multiple time and return the lowest number out of all the times it run
def check_sensor(times, sensor_num):
    dis = Distance(sensor_num)
    for x in range(times):
        temp = Distance(sensor_num)
        if(temp < dis):
            dis = temp
    return dis

#use to run the FWDKIN equation and update the location of the car
def update_location(start_time, local_x, local_y, local_radians, origin_radians):
    VL , VR = wheelspeed()
    
    #Calculate sameple time for foward kinematics
    sample_time = time.time() - start_time
    
    
    # distance between left and right wheel = 0.229m
    #Updates current location of car.
    ans =  FWDKIN(local_x, local_y, (local_radians - origin_radians), sample_time, 0.229, VL, VR)
    start_time = time.time()
    x = ans[0][0]
    y = ans[1][0]
    radian = origin_radians + ans[2][0]
    return start_time, x,y,radian

#the state where using the pid function to do simple movement to move toward the destination
#This case handle when there is no obstacle
def default_state(x,y,local_x, local_y,sensor, leftspeed, rightspeed,prev,sumError,start_time,origin_radians):


    if Distance(0) < 0.6:
        print("Hit wall")
        
    
    if x == 0:		#handle the special case of x = 0
        x = 0.000001
    
    #Read new angle
    local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )
    
    #Prevents angle from being too large due to invalid data.
    while math.degrees(local_radians) < -500 or math.degrees(local_radians) > 500:
        local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )
    
    #Calculate desired angle to reach target.
    t_angle = math.degrees ( math.atan( (y - local_y)/(x - local_x) ) )
    
    if t_angle > 0:
        #Quadrant 1
        if (y - local_y) > 0:
            t_angle = t_angle		
        
        #Quadrant 2
        else:
            t_angle = 180 + t_angle
            
    else:	
        #Quadrant 4
        if (y - local_y) < 0:
            t_angle = 360 + t_angle
            
        #Quadrant 3
        else:
            t_angle = 180 + t_angle
    
    #This is our target angle. The angle we desire to reach.		
    angle = (t_angle + math.degrees(local_radians)) % 360	
    
    
    print("Target angle: {}" .format(angle))
    print("Current angle: {}" .format(math.degrees(local_radians)))
    

    output, prev, sumError = pid(angle, mirror_sensor_angle( sensor.euler[0] ), time.time() - start_time,  prev, sumError, .05, .012, .011) # .05 .012 .011
    
    rightspeed = max(0.3, min(1,rightspeed + output))
    leftspeed = max(0.3, min(1,leftspeed - output))
    control_speed(leftspeed, rightspeed)
    
    

    start_time, local_x, local_y, local_radians = update_location(start_time, local_x, local_y, local_radians, origin_radians)
    
    #Update distance magnitude from current location to destination.
    total_distance = math.sqrt((x - local_x)**2 + (y - local_y)**2)

    print("distance need to travel: {} ".format(total_distance))
    print("The current x and y is {}, {}".format(local_x, local_y))
    print("Foward Kinematic angle: {}".format(math.degrees(local_radians)))
    print()
    return start_time , local_x, local_y, local_radians, leftspeed, rightspeed, prev, sumError


#state to turn right when sense an obstacle
def turn_right_state(local_x, local_y, sensor,origin_radians):
    #turn for a short amount of time like 0.2 second
    # idea: turn until the front sensor do not sense the obstacle?

    turn_right()

    current_time = time.time()

    time.sleep(0.5)

    #Read new angle
    local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )
    
    #Prevents angle from being too large due to invalid data.
    while math.degrees(local_radians) < -500 or math.degrees(local_radians) > 500:
        local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )

    start_time, local_x, local_y, local_radians = update_location(current_time, local_x, local_y, local_radians, origin_radians)
    return start_time , local_x, local_y, local_radians


#state to turn left when sense an obstacle
def turn_left_state(local_x, local_y, sensor,origin_radians):
    turn_left()

    current_time = time.time()

    time.sleep(0.5)

    #Read new angle
    local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )

    #Prevents angle from being too large due to invalid data.
    while math.degrees(local_radians) < -500 or math.degrees(local_radians) > 500:
        local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )

    start_time, local_x, local_y, local_radians = update_location(current_time, local_x, local_y, local_radians, origin_radians)
    return start_time , local_x, local_y, local_radians

#wall follow right state
def wall_follow_right_state(local_x, local_y, sensor,origin_radians, start_time, prev, sumError, leftspeed, rightspeed):
    target_distance_wall = 0.4      #need to adjust later, a variable for the target distance to the wall


    output, prev, sumError = pid_wall(target_distance_wall, Distance(4), time.time() - start_time,  prev, sumError, 0.15, 0.036, 0.33) #Call the pid function to obtain the change needed for the motor

    #Read new angle
    local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )
    
    #Prevents angle from being too large due to invalid data.
    while math.degrees(local_radians) < -500 or math.degrees(local_radians) > 500:
        local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )

    start_time, local_x, local_y, local_radians = update_location(start_time, local_x, local_y, local_radians, origin_radians)


    #update the motor speed
    rightspeed = max(0.3, min(1,rightspeed - output))
    leftspeed = max(0.3, min(1,leftspeed + output))

    control_speed(leftspeed, rightspeed)
    time.sleep(.05)
    return start_time , local_x, local_y, local_radians, prev, sumError, leftspeed, rightspeed
    

#wall follow left state
def wall_follow_left_state(local_x, local_y, sensor,origin_radians, start_time, prev, sumError, leftspeed, rightspeed):
    target_distance_wall = 0.4      #need to adjust later, a variable for the target distance to the wall


    output, prev, sumError = pid_wall(target_distance_wall, Distance(3), time.time() - start_time,  prev, sumError, 0.15, 0.036, 0.33) #Call the pid function to obtain the change needed for the motor

    #Read new angle
    local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )
    
    #Prevents angle from being too large due to invalid data.
    while math.degrees(local_radians) < -500 or math.degrees(local_radians) > 500:
        local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )

    start_time, local_x, local_y, local_radians = update_location(start_time, local_x, local_y, local_radians, origin_radians)


    #update the motor speed
    rightspeed = max(0.3, min(1,rightspeed - output))
    leftspeed = max(0.3, min(1,leftspeed + output))

    control_speed(leftspeed, rightspeed)
    time.sleep(.05)
    return start_time , local_x, local_y, local_radians, prev, sumError, leftspeed, rightspeed
    

#CRASH state
def crash_state():
    control_speed(None, None)
    print("The car is going to crash")
    return


#need to finish later if need this function
def destination_right(x,y,local_x,local_y,sensor):
    #Read new angle
    local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )
    
    #Prevents angle from being too large due to invalid data.
    while math.degrees(local_radians) < -500 or math.degrees(local_radians) > 500:
        local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )

    c_angle = math.degrees(local_radians)

    t_angle = math.degrees ( math.atan( (y - local_y)/(x - local_x) ) )

    if(c_angle >= 180):
        if( (t_angle > (c_angle - 180)) and ((t_angle < c_angle))):
            return True
        else:
            return False

    else:
        if( (t_angle > (360 + c_angle - 180)) or ((t_angle < c_angle))):
            return True
        else:
            return False


#need to finish later if need this function
def destination_left(x,y,local_x,local_y,sensor):
    #Read new angle
    local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )
    
    #Prevents angle from being too large due to invalid data.
    while math.degrees(local_radians) < -500 or math.degrees(local_radians) > 500:
        local_radians = math.radians(mirror_sensor_angle( sensor.euler[0] ) )

    c_angle = math.degrees(local_radians)

    t_angle = math.degrees ( math.atan( (y - local_y)/(x - local_x) ) )

    if(c_angle <= 180):
        if( (t_angle <= (c_angle + 180)) and ((t_angle > c_angle))):
            return True
        else:
            return False

    else:
        if( (t_angle < (c_angle - 180)) or ((t_angle > c_angle))):
            return True
        else:
            return False

#the car reached the destination, stop the car and print a statement that indicate finish
def reach_destinition_state():
    control_speed(None, None)
    print("Reached destination, yeah!!!")
    return

def change_state(curr_state, x,y,local_x,local_y, prev, sumError):
    total_distance = math.sqrt((x - local_x)**2 + (y - local_y)**2)

    temp_state = state.NONE
    
    
    if(total_distance < 0.2):
        temp_state = state.REACH_DESTINATION
        
    elif(Distance(0) < 0.3):
        temp_state = state.CRASH
        
    elif((Distance(0) < 1.5) and (Distance(1) < 1.5)):
        temp_state = state.TURN_RIGHT

    elif((Distance(0) < 1.5) and (Distance(2) < 1.5)):
        temp_state = state.TURN_LEFT

    elif(destination_left(x,y,local_x,local_y,sensor) and (Distance(3) < 0.6)):
        temp_state = state.WALL_LEFT

    elif(destination_right(x,y,local_x,local_y,sensor) and (Distance(4) < 0.6)):
        temp_state = state.WALL_RIGHT
        
    else:
        temp_state = state.DEFAULT

    #if state change, reset prev and suError back to 0 for pid function
    if(curr_state != temp_state):
        curr_state = temp_state
        prev = 0
        sumError = 0

    return curr_state, prev, sumError

def path_planning3(x,y, sensor):
    #Used to store current x y location
    local_x = 0
    local_y = 0
    	
    #Initialize list for graph
    x_list = []
    y_list = []
    angle_list = []	
    time_list = []

    begin_time = time.time()
    
    #Read the current angle we are facing
    local_radians = math.radians(mirror_sensor_angle(sensor.euler[0]))
    	
    #This is the initial angle we are facing
    origin_radians = local_radians
    
    #Initialize speed for L/R motors
    leftspeed = 1
    rightspeed = 1
    
    #Initialize PID
    prev = 0
    sumError = 0
    start_time = time.time()
    
    	
    curr_state = state.DEFAULT
	
    #Car will keep driving until it is less than .2m from target
    while (curr_state != state.REACH_DESTINATION):
        print("Distance 0: {}".format(Distance(0)))
        print("Distance 1: {}".format(Distance(1)))
        print("Distance 2: {}".format(Distance(2)))
        
        if(curr_state == state.DEFAULT):
            print("current state is {}".format(curr_state))
            start_time , local_x, local_y, local_radians, leftspeed, rightspeed, prev, sumError = default_state(x,y,local_x, local_y,sensor, leftspeed, rightspeed,prev,sumError,start_time,origin_radians)

            curr_state, prev, sumError = change_state(curr_state, x,y,local_x,local_y, prev, sumError)
            print("Next state is {}".format(curr_state))
            
        elif(curr_state == state.CRASH):
            crash_state()
            return x_list,y_list,angle_list,time_list
        


        elif(curr_state == state.TURN_RIGHT):
            print("current state is {}".format(curr_state))
            start_time , local_x, local_y, local_radians = turn_right_state(local_x, local_y, sensor,origin_radians)

            curr_state, prev, sumError = change_state(curr_state, x,y,local_x,local_y, prev, sumError)
            print("Next state is {}".format(curr_state))

        elif(curr_state == state.TURN_LEFT):
            print("current state is {}".format(curr_state))
            start_time , local_x, local_y, local_radians = turn_left_state(local_x, local_y, sensor,origin_radians)

            curr_state, prev, sumError = change_state(curr_state, x,y,local_x,local_y, prev, sumError)
            print("Next state is {}".format(curr_state))

        elif(curr_state == state.WALL_LEFT):
            print("current state is {}".format(curr_state))
            curr_state, local_x, local_y, local_radians, prev, sumError, leftspeed, rightspeed = wall_follow_left_state(local_x, local_y, sensor,origin_radians, start_time, prev, sumError, leftspeed, rightspeed)

            curr_state, prev, sumError = change_state(curr_state, x,y,local_x,local_y, prev, sumError)
            print("Next state is {}".format(curr_state))

        elif(curr_state == state.WALL_RIGHT):
            print("current state is {}".format(curr_state))
            start_time , local_x, local_y, local_radians, prev, sumError, leftspeed, rightspeed = wall_follow_right_state(local_x, local_y, sensor,origin_radians, start_time, prev, sumError, leftspeed, rightspeed)

            curr_state, prev, sumError = change_state(curr_state, x,y,local_x,local_y, prev, sumError)
            print("Next state is {}".format(curr_state))

        #Append data points
        x_list.append(local_x)
        y_list.append(local_y)
        angle_list.append(math.degrees(local_radians))
        time_list.append(time.time() - begin_time)

    reach_destinition_state()
    return x_list,y_list,angle_list,time_list



i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c) 
control_speed(None,None)
# ~ sensor = calibration()
while True:
	x = input("number of x ")
	y = input("number of y ")
	path_planning3(float(x),float(y), sensor)
            
