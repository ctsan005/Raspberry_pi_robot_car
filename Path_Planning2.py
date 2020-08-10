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

def path_planning2(x,y, sensor):
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

        # read 3 sensors to identify is there obstacle and use them to determine which case to go in
        left_obstacle = (Distance(1) < 1.5)
        middle_obstacle = (Distance(0) < 1.5)
        right_obstacle = (Distance(2) < 1.5)

        # case #1: Default case. 
        # Use pid function to face the target and continue to approch the target
        while(middle_obstacle == False):

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
                    
                #Quadrant 3
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

        # read 3 sensors to identify is there obstacle and use them to determine which case to go in
        left_obstacle = (Distance(1) < 1.5)
        middle_obstacle = (Distance(0) < 1.5)
        right_obstacle = (Distance(2) < 1.5)


        #case 2: turn right and pass the obstacle
        #identify an obstacle so need to turn right, if it is a long obstacle, use wall following to following it until it past the obstacle
        #important: need to find a way to update the location within this case
        if((left_obstacle == True) and (middle_obstacle == True) ):
            
            #need to read the ir sensor first and use to determine is the obstacle a long obstacle
            check_dis = Distance(3)

            #turn for a short amount of time like 0.2 second
            # idea: turn until the front sensor do not sense the obstacle?

            turn_right()

            current_time = time.time()

            time.sleep(0.5)

            VL , VR = wheelspeed()

            ans =  FWDKIN(local_x, local_y, (math.radians(mirror_sensor_angle(sensor.euler[0])) - origin_radians), (time.time() - current_time), 0.229, VL, VR)
            local_x = ans[0][0]
            local_y = ans[1][0]
            local_radians = origin_radians + ans[2][0]

            start_time = time.time()



            #read the ir sensor mulitiple times and get the losest number
            new_dis = check_dis
            for x in range(3):
                temp = Distance(3)
                if(temp < new_dis):
                    new_dis = temp

            #if the new dis is smaller than check_dis, that mean turning right did not avoid the obstacle yet, need to wall follow it to past it
            if((check_dis - new_dis) > 0.2):
                
                #init a variable to keep track the distance of the car and the obstacle around it
                distance = 1

                #read the sensor 3 times and get the smallest number to prevent one faluty number
                for x in range(3):
                    temp = Distance(3)
                    if(temp < distance):
                        distance = temp

                start_time = time.time()
                #while the distance is less than 1, continue to follow the wall until past the obstacle
                while(distance < 0.7):
                    

                    target_distance_wall = 0.4      #need to adjust later, a variable for the target distance to the wall


                    #init the variable for pid
                    prev = 0
                    sumError = 0


                    output, prev, sumError = pid_wall(target_distance_wall, Distance(3), time.time() - start_time,  prev, sumError, 0.15, 0.036, 0.33) #Call the pid function to obtain the change needed for the motor

                    VL , VR = wheelspeed()

                    ans =  FWDKIN(local_x, local_y, (math.radians(mirror_sensor_angle(sensor.euler[0])) - origin_radians), (time.time() - current_time), 0.229, VL, VR)
                    local_x = ans[0][0]
                    local_y = ans[1][0]
                    local_radians = origin_radians + ans[2][0]

                    start_time = time.time()


                    #update the motor speed
                    rightspeed = max(0.3, min(1,rightspeed - output))
                    leftspeed = max(0.3, min(1,leftspeed + output))

                    control_speed(leftspeed, rightspeed)
                    time.sleep(.05)

                    #read the sensor 3 times and get the smallest number to prevent one faluty number
                    distance = Distance(3)
                    for x in range(3):
                        temp = Distance(3)
                        if(temp < distance):
                            distance = temp



        #case 3: turn left and pass the obstacle
        #identify an obstacle so need to turn left, if it is a long obstacle, use wall following to following it until it past the obstacle
        #important: need to find a way to update the location within this case
        if((right_obstacle == True) and (middle_obstacle == True) ):
            
            #need to read the ir sensor first and use to determine is the obstacle a long obstacle
            check_dis = Distance(4)

            #turn for a short amount of time like 0.2 second
            # idea: turn until the front sensor do not sense the obstacle?

            turn_left()

            current_time = time.time()

            time.sleep(0.5)

            VL , VR = wheelspeed()

            ans =  FWDKIN(local_x, local_y, (math.radians(mirror_sensor_angle(sensor.euler[0])) - origin_radians), (time.time() - current_time), 0.229, VL, VR)
            local_x = ans[0][0]
            local_y = ans[1][0]
            local_radians = origin_radians + ans[2][0]

            start_time = time.time()



            #read the ir sensor mulitiple times and get the losest number
            new_dis = check_dis
            for x in range(4):
                temp = Distance(4)
                if(temp < new_dis):
                    new_dis = temp

            #if the new dis is smaller than check_dis, that mean turning right did not avoid the obstacle yet, need to wall follow it to past it
            if((check_dis - new_dis) > 0.2):
                
                #init a variable to keep track the distance of the car and the obstacle around it
                distance = 1

                #read the sensor 3 times and get the smallest number to prevent one faluty number
                for x in range(3):
                    temp = Distance(4)
                    if(temp < distance):
                        distance = temp

                start_time = time.time()
                #while the distance is less than 1, continue to follow the wall until past the obstacle
                while(distance < 0.7):
                    

                    target_distance_wall = 0.4      #need to adjust later, a variable for the target distance to the wall


                    #init the variable for pid
                    prev = 0
                    sumError = 0


                    output, prev, sumError = pid_wall(target_distance_wall, Distance(3), time.time() - start_time,  prev, sumError, 0.15, 0.036, 0.33) #Call the pid function to obtain the change needed for the motor

                    VL , VR = wheelspeed()

                    ans =  FWDKIN(local_x, local_y, (math.radians(mirror_sensor_angle(sensor.euler[0])) - origin_radians), (time.time() - current_time), 0.229, VL, VR)
                    local_x = ans[0][0]
                    local_y = ans[1][0]
                    local_radians = origin_radians + ans[2][0]

                    start_time = time.time()


                    #update the motor speed
                    rightspeed = max(0.3, min(1,rightspeed - output))
                    leftspeed = max(0.3, min(1,leftspeed + output))

                    control_speed(leftspeed, rightspeed)
                    time.sleep(.05)

                    #read the sensor 3 times and get the smallest number to prevent one faluty number
                    distance = Distance(3)
                    for x in range(3):
                        temp = Distance(3)
                        if(temp < distance):
                            distance = temp