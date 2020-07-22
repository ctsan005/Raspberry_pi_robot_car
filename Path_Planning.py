from Ultrasonic_Sensor import *
import math
import time
from calibration import calibration
from motorFunction import pid
from Foward_Kinematics import FWDKIN
from Encoder import *
import board
import busio
import adafruit_bno055
from adafruit_motorkit import MotorKit
import matplotlib.pyplot as plt 

Left = 1 # for the left sensor
Right = 2 # for the right sensor
kit = MotorKit()

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

def path_planning(x,y, sensor):
        
    #Used to store current x y location
    local_x = 0
    local_y = 0
    	
    #Initialize list for graph
    x_list = []
    y_list = []
    angle_list = []	
    
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
    	
    #Used to check if the car is already moving.
    check = 0
    
    #Calculates distance magnitude from current location to target. 
    total_distance = math.sqrt(x**2 + y**2)
    	

	
    #Car will keep driving until it is less than .2m from target
    while (total_distance > 0.2):
		
        if Distance(0) < 0.6:
            print("Hit wall")
            break
        
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
				
			#Quadrant 2
            else:
                t_angle = 180 + t_angle
		
        #This is our target angle. The angle we desire to reach.		
        angle = (t_angle + math.degrees(local_radians)) % 360	
		
		
        print("Target angle: {}" .format(angle))
        print("Current angle: {}" .format(math.degrees(local_radians)))
		
        #Starts timers. Car will begin to move for the first time. 
        if(check == 0):
            #Time for foward kinematics
            start_time = time.time()
            #Time for PID iteration time
            PID_loop_time = time.time()
            check = 1

        output, prev, sumError = pid(angle, mirror_sensor_angle( sensor.euler[0] ), time.time() - PID_loop_time,  prev, sumError, .05, .012, .011) # .05 .012 .011
		
        rightspeed = max(0.3, min(1,rightspeed + output))
        leftspeed = max(0.3, min(1,leftspeed - output))
        control_speed(leftspeed, rightspeed)
        VL , VR = wheelspeed()
        
        #Iteration time for PID
        PID_loop_time = time.time()
		
        #Calculate sameple time for foward kinematics
        sample_time = time.time() - start_time
        start_time = time.time()
        
		# distance between left and right wheel = 0.229m
        #Updates current location of car.
        ans =  FWDKIN(local_x, local_y, (local_radians - origin_radians), sample_time, 0.229, VL, VR)
        local_x = ans[0][0]
        local_y = ans[1][0]
        local_radians = origin_radians + ans[2][0]
		
        #Update distance magnitude from current location to destination.
        total_distance = math.sqrt((x - local_x)**2 + (y - local_y)**2)

        print("distance need to travel: {} ".format(total_distance))
        print("The current x and y is {}, {}".format(local_x, local_y))
        print("Foward Kinematic angle: {}".format(math.degrees(local_radians)))
        print()
		
        #Append data points
        x_list.append(local_x)
        y_list.append(local_y)
        angle_list.append(math.degrees(local_radians))
		
		
		# ~ if DLeft < 2 or DRight < 2:
			# ~ if DLeft < DRight:
				# ~ if distance < DLeft: # go straight
				# ~ else:
					# ~ # turn right
				
			# ~ elif DRight < DLeft:
				# ~ if distance < DRight: # go straight
				# ~ else:
					# ~ # turn left
				
			# ~ else:
				# ~ if distance < DRight: # go straight
				# ~ else:
					# ~ # just turn left
				
			# update distance
        
    #Stop Motors
    control_speed(None, None)
	
    if(Distance(0) <= 0.6):
        print("hit wall")
    else:
        print("reach destination")
        return x_list, y_list, angle_list


i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c) 
control_speed(None,None)
# ~ sensor = calibration()
while True:
	x = input("number of x ")
	y = input("number of y ")
	x_list, y_list, angle_list = path_planning(float(x),float(y), sensor)
	
	x_y_graph(x_list, y_list)
	x_y_graph2(x_list, y_list)
	angle_graph(angle_list)
	
	
	
	
	
	# ~ path_planning(1,2, sensor)
	
# ~ control_speed(None,None)
# ~ time.sleep(2)
# ~ control_speed(1,1)
# ~ time.sleep(0.25)
# ~ control_speed(0.3,1)
# ~ print("speed change")
# ~ time.sleep(2)
# ~ control_speed(None,None)
# ~ while True:
	# ~ print(Distance(0))
