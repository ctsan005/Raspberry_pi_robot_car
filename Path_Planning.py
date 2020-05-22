from Ultrasonic_Sensor import *
import math
import time
from Tune_PID import calibration
from motorFunction import pid
from Foward_Kinematics import FWDKIN
from Encoder import wheelspeed

Left = 1 # for the left sensor
Right = 2 # for the right sensor

def control_speed(left, right):
	kit.motor1.throttle = left
	kit.motor2.throttle = left
	kit.motor3.throttle = right
	kit.motor4.throttle = right

def path_planning(x,y, sensor):
	
	local_x = 0
	local_y = 0		#use to store the current lcoation
	
	local_radian = math.radians(sensor.euler[0])	#read the current facing angle
	
	leftspeed = 1
	rightspeed = 1		#init the speed for left and right motor
	
	prev = 0
	sumError = 0
	velocity = 0		#use for going straight function
	
	check = 0			#use to keep track did the car moved already		
	
	
	total_distance = math.sqrt(x**2 + y**2) # total distance to travel to target, use to see do we need to adjust the direct to avoid object
	distance = total_distance	#init the distance to target = total distance
	
	while total_distance > 0.2: # check does it reach the distance yet
		
		# ~ DLeft = Distance(Left) # get the left sensor distance
		# ~ DRight = Distance(Right) # get the right sensor distance
		
		if x == 0:		#handle the special case of x = 0
			x = 0.000001
			
		angle = (math.degrees(math.atan(y - local_y/x - local_x)) + math.degrees(local_radian)) % 360 # get the angle toward the destination 
		print("current angle: {}" .format(angle))
		# move to face the correct angle
		# ~ if(abs(angle - local_angle) > 3):
		
		
			
		
		
		local_radian = math.radians(sensor.euler[0])
		
		while math.degrees(local_radian) < -500:			#handle noise from sensor
			local_radian = math.radians(sensor.euler[0])
			
		control_speed(leftspeed, rightspeed)		#control the speed of the car
		
		if(check == 0):			#start the timer after the car move for the first time
			start_time = time.time()
			check = 1
		
		
		# go straight
		# ~ if DLeft > 2 and DRight > 2:
		rightspeed, leftspeed, prev, sumError = pid(angle, sensor, rightspeed, leftspeed, prev, sumError, .5, .01, .2)
		control_speed(leftspeed, rightspeed)
		
		VL , VR = wheelspeed()
		sample_time = time.time() - start_time
		start_time = time.time()
		# distance between left and right wheel = 0.229m
		ans =  FWDKIN(local_x, local_y, local_radian, sample_time, 0.229, VL, VR)
		
		local_x = ans[0][0]
		local_y = ans[1][0]
		local_radians = ans[2][0]
		
		total_distance = math.sqrt((x - local_x)**2 + (y - local_y)**2)		#update the total distance to the destination
		
		
		#update the local x and y number
		# ~ local_x = math.cos(math.radians(angle)) * dis_travel + local_x
		# ~ local_y = math.sin(math.radians(angle)) * dis_travel + local_y
		# use another function to help update local x and y and angle
		
		
		
		print("distance need to travel: {} ".format(total_distance))
		
		
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
				
		
	control_speed(None, None)		
	print("reach destination")
	

sensor = adafruit_bno055.BNO055(i2c) 
control_speed(None,None)
# ~ sensor = calibration()
while True:
	x = input("number of x ")
	y = input("number of y ")
	path_planning(float(x),float(y), sensor)
