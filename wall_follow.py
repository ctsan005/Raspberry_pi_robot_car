import time
import board
import busio
from subprocess import call
from motorFunction import pid
from Ultrasonic_Sensor import *
import adafruit_bno055
from adafruit_motorkit import MotorKit


kit = MotorKit()
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

# ~ sensor = calibration() #calibrate sensor
sensor = adafruit_bno055.BNO055(i2c) 

def wall_following(kp,ki,kd,rightspeed,leftspeed):

	start_time = time.time()

	target_distance_wall = 10      #need to adjust later, a variable for the target distance to the wall

	#init the car speed
	kit.motor1.throttle = leftspeed
	kit.motor2.throttle = leftspeed
	kit.motor3.throttle = rightspeed
	kit.motor4.throttle = rightspeed


	#graph data
	t = 0
	Time = [] #Store Data for time
	wall_dis = [] #Store current angle at time t
	Target = [] #Store Target data as a reference


	#use for pid function to tell the iteration time for the pid funciton
	loop_time = time.time()

	while( (time.time() - start_time) < 10): 
		output, prev, sumError = pid(target_distance_wall, Distance(1), time.time() - loop_time,  prev, sumError, kp, ki, kd) #Call the pid function to obtain the change needed for the motor
		
		# ~ if prev > 2:
			# ~ sumError = 0
		
		loop_time = time.time()			#update the iteration time
		
		#update the motor speed
		rightspeed = max(0.3, min(1,rightspeed + output))
		leftspeed = max(0.3, min(1,leftspeed - output))
		
		print("The rightspeed and leftspeed is: {}, {}".format(rightspeed, leftspeed))
		kit.motor1.throttle = leftspeed
		kit.motor2.throttle = leftspeed
		kit.motor3.throttle = rightspeed
		kit.motor4.throttle = rightspeed

		#update the graph
		wall_dis.append(t)
		Time.append(t)
		Target.append(t)
		wall_dis[t] = Distance(1)
		Time[t] = time.time() - start_time
		Target[t] = target_distance_wall
		t = t + 1

		#sleep to prevent the pid run too fast
		time.sleep(.02)

		print(Distance(0))

	#stop the car to prevent the car to hit the wall
	kit.motor1.throttle = None
	kit.motor2.throttle = None
	kit.motor3.throttle = None
	kit.motor4.throttle = None

