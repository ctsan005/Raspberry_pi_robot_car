from Ultrasonic_Sensor import *
from Straight_Car import *
import math

Left = 1 # for the left sensor
Right = 2 # for the right sensor

def path_planning(x,y):
	
	total_distance = x^2 + y^2 # total distance to travel to target, use to see do we need to adjust the direct to avoid object
	distance = total_distance	#init the distance to target = total distance
	while total_distance > 0.2: # check does it reach the distance yet
		DLeft = Distance(Left) # get the left sensor distance
		DRight = Distance(Right) # get the right sensor distance
		angle = math.degrees(math.atan(y/x)) # get the angle 
		# move to face the correct angle
		
		
		# go straight
		while DLeft < 2 or DRight < 2:
			if DLeft < DRight:
				if distance < DLeft: # go straight
				else:
					# turn right
				
			elif DRight < DLeft:
				if distance < DRight: # go straight
				else:
					# turn left
				
			else:
				if distance < DRight: # go straight
				else:
					# just turn left
				
			# update distance
				
		
			
	print("reach destination")
			
