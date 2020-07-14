import time
import board
import busio
from subprocess import call
from motorFunction import *
from Tune_PID import *
from Ultrasonic_Sensor import *
import adafruit_bno055
from adafruit_motorkit import MotorKit

kit = MotorKit()
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

# ~ sensor = calibration() #calibrate sensor
sensor = adafruit_bno055.BNO055(i2c) 

#use for testing the pid function with two parts, first going straight and then make a turn by setting the target angle slightly different. It will generate a graph for the angle the car that it faced vs the target of the angle that it should face.
def test_PID(kp,ki,kd):
	start_time = time.time() 			#use to keep track the begin time of the straight part
	target = mirror_sensor_angle ( sensor.euler[0] )		#read the sensor data and convert it to unit circle format angle
	print("initial target is : {}" .format(target))

	#pid data
	prev = 0
	sumError = 0

	#speed of car
	rightspeed = 1
	leftspeed = 1

	#init the car speed
	kit.motor1.throttle = leftspeed
	kit.motor2.throttle = leftspeed
	kit.motor3.throttle = rightspeed
	kit.motor4.throttle = rightspeed

	#graph data
	t = 0
	Time = [] #Store Data for time
	Angle = [] #Store current angle at time t
	Target = [] #Store Target data as a reference
	
	#use for pid function to tell the iteration time for the pid funciton
	loop_time = time.time()

	#continue to go straight until 0.5 sec or there is a obstacle that is closer than 0.6 distance from the car
	while( (time.time() - start_time)<.5): 
		if(Distance(0) < .6):
			print("hit wall")
			break

		
		output, prev, sumError = pid(target, mirror_sensor_angle( sensor.euler[0] ), time.time() - loop_time,  prev, sumError, kp, ki, kd)		#call the pid function to calculate the speed change required
		
		loop_time = time.time()			#update the iteration time for the pid
		
		#update the speed of the left and right motor
		rightspeed = max(0.3, min(1,rightspeed + output))
		leftspeed = max(0.3, min(1,leftspeed - output))
		
		print("The rightspeed and leftspeed is: {}, {}".format(rightspeed, leftspeed))
		
		kit.motor1.throttle = leftspeed
		kit.motor2.throttle = leftspeed
		kit.motor3.throttle = rightspeed
		kit.motor4.throttle = rightspeed

		#update the graph data
		Angle.append(t)
		Time.append(t)
		Target.append(t)
		Angle[t] = mirror_sensor_angle( sensor.euler[0] )
		Time[t] = time.time() - start_time
		Target[t] = target
		t = t + 1

		#sleep to prevent the pid function run too fast
		time.sleep(.02)

	print("finish straight")
	print("target is : {}" .format(mirror_sensor_angle( sensor.euler[0] ) ))
	# ~ start_time = time.time()

	#Change the target angle to achieve turning, adding 0 indicate continue to go straight
	target = (target + 0)%360
	print(Distance(0))

	#continue to go straight until reach close to the wall
	while( Distance(0) > .6): 
		output, prev, sumError = pid(target, mirror_sensor_angle( sensor.euler[0] ), time.time() - loop_time,  prev, sumError, kp, ki, kd) #Call the pid function to obtain the change needed for the motor
		
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
		Angle.append(t)
		Time.append(t)
		Target.append(t)
		Angle[t] = mirror_sensor_angle( sensor.euler[0] )
		Time[t] = time.time() - start_time
		Target[t] = target
		t = t + 1

		#sleep to prevent the pid run too fast
		time.sleep(.02)

		print(Distance(0))

	#stop the car to prevent the car to hit the wall
	kit.motor1.throttle = None
	kit.motor2.throttle = None
	kit.motor3.throttle = None
	kit.motor4.throttle = None

	#show message the car finish the turning part of the function
	print("finish turning")
	print("target is : {}" .format(mirror_sensor_angle( sensor.euler[0] ) ))

	#call the graph function to generate the graph
	graph(Time, Angle, Time, Target)
	

#testing the function
while True:
	kit.motor1.throttle = None			#Goof kp ki kd value are 0.01, 0.005, 0.01
	kit.motor2.throttle = None
	kit.motor3.throttle = None
	kit.motor4.throttle = None
	print(Distance(0))
	kp = input("please enter kp value: ")
	ki = input("please enter ki value: ")
	kd = input("please enter kd value: ")
	test_PID(float(kp),float(ki),float(kd))
