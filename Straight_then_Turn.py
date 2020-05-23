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


def test_PID(kp,ki,kd):
	# ~ input("press enter to start")
	start_time = time.time() 
	target = mirror_sensor_angle ( sensor.euler[0] )
	print("initial target is : {}" .format(target))
	prev = 0
	sumError = 0
	rightspeed = 1
	leftspeed = 1
	kit.motor1.throttle = leftspeed
	kit.motor2.throttle = leftspeed
	kit.motor3.throttle = rightspeed
	kit.motor4.throttle = rightspeed
	t = 0
	Time = [] #Store Data for time
	Angle = [] #Store current angle at time t
	Target = [] #Store Target data as a reference
	
	loop_time = time.time()

	while( (time.time() - start_time)<.5): 
		if(Distance(0) < .6):
			print("hit wall")
			break
		output, prev, sumError = pid(target, mirror_sensor_angle( sensor.euler[0] ), time.time() - loop_time,  prev, sumError, kp, ki, kd)
		
		loop_time = time.time()
		
		rightspeed = max(0.3, min(1,rightspeed + output))
		leftspeed = max(0.3, min(1,leftspeed - output))
		
		print("The rightspeed and leftspeed is: {}, {}".format(rightspeed, leftspeed))
		
		kit.motor1.throttle = leftspeed
		kit.motor2.throttle = leftspeed
		kit.motor3.throttle = rightspeed
		kit.motor4.throttle = rightspeed
		Angle.append(t)
		Time.append(t)
		Target.append(t)
		Angle[t] = mirror_sensor_angle( sensor.euler[0] )
		Time[t] = time.time() - start_time
		Target[t] = target
		t = t + 1
		time.sleep(.02)

	print("finish straight")
	print("target is : {}" .format(mirror_sensor_angle( sensor.euler[0] ) ))
	# ~ start_time = time.time()
	target = (target + 0)%360
	print(Distance(0))
	count = 0
	while( Distance(0) > .6): #time.time() - start_time < 4
		output, prev, sumError = pid(target, mirror_sensor_angle( sensor.euler[0] ), time.time() - loop_time,  prev, sumError, kp, ki, kd) #.5, .005, .4
		
		# ~ if prev > 2:
			# ~ sumError = 0
		
		loop_time = time.time()
		
		rightspeed = max(0.3, min(1,rightspeed + output))
		leftspeed = max(0.3, min(1,leftspeed - output))
		
		print("The rightspeed and leftspeed is: {}, {}".format(rightspeed, leftspeed))
		kit.motor1.throttle = leftspeed
		kit.motor2.throttle = leftspeed
		kit.motor3.throttle = rightspeed
		kit.motor4.throttle = rightspeed
		Angle.append(t)
		Time.append(t)
		Target.append(t)
		Angle[t] = mirror_sensor_angle( sensor.euler[0] )
		Time[t] = time.time() - start_time
		Target[t] = target
		t = t + 1
		time.sleep(.02)
		print(Distance(0))

		
	kit.motor1.throttle = None
	kit.motor2.throttle = None
	kit.motor3.throttle = None
	kit.motor4.throttle = None
	print("finish turning")
	print("target is : {}" .format(mirror_sensor_angle( sensor.euler[0] ) ))
	graph(Time, Angle, Time, Target)
	
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
