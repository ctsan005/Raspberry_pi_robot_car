from Ultrasonic_Sensor import *
from Straight_Car import *
import math
import time
from Tune_PID import calibration
from Velocity_Distance_Calculation import velocity_calculation

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
	local_angle = sensor.euler[0]
	leftspeed = 1
	rightspeed = 1
	prev = 0
	sumError = 0
	velocity = 0
	check = 0
	
	
	total_distance = math.sqrt(x**2 + y**2) # total distance to travel to target, use to see do we need to adjust the direct to avoid object
	distance = total_distance	#init the distance to target = total distance
	while total_distance > 0.2: # check does it reach the distance yet
		
		# ~ DLeft = Distance(Left) # get the left sensor distance
		# ~ DRight = Distance(Right) # get the right sensor distance
		if x == 0:
			x = 0.000001
		angle = (math.degrees(math.atan(y/x)) + local_angle) % 360 # get the angle 
		print("current angle: {}" .format(angle))
		# move to face the correct angle
		# ~ if(abs(angle - local_angle) > 3):
		
		if(check == 0):
			rotate_in_place(sensor, angle-local_angle)
			check = 1
		start_time = time.time()
		
		local_angle = sensor.euler[0]
		
		while local_angle < -500:
			local_angle = sensor.euler[0]
		control_speed(leftspeed, rightspeed)
		
		
		# go straight
		# ~ if DLeft > 2 and DRight > 2:
		rightspeed, leftspeed, prev, sumError = pid(angle, sensor, rightspeed, leftspeed, prev, sumError, .5, .005, .4)
		control_speed(leftspeed, rightspeed)
		sample_time = time.time() - start_time
		dis_travel = velocity_calculation(velocity, sensor, sample_time)
		total_distance = total_distance - dis_travel
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
