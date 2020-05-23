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

input("press enter to start")
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

while( time.time() - start_time<5): 
	if(Distance(0) < .6):
		print("hit wall")
		break
	rightspeed, leftspeed, prev, sumError = pid(target, sensor, rightspeed, leftspeed, prev, sumError, .5, .01, .2)
	kit.motor1.throttle = leftspeed
	kit.motor2.throttle = leftspeed
	kit.motor3.throttle = rightspeed
	kit.motor4.throttle = rightspeed
	Angle.append(t)
	Time.append(t)
	Target.append(t)
	Angle[t] = mirror_sensor_angle( sensor.euler[0] )
	Time[t] = t
	Target[t] = target
	t = t + 1
	time.sleep(.02)

print("finish straight")
# ~ print("target is : {}" .format(mirror_sensor_angle( sensor.euler[0] ) ))
# ~ start_time = time.time()
# ~ target = target + 0
# ~ print(Distance(0))
# ~ while( Distance(0) > .6): #time.time() - start_time < 4
	# ~ rightspeed, leftspeed, prev, sumError = pid(target, sensor, rightspeed, leftspeed, prev, sumError, .5, .01, .2) #.5, .005, .4
	# ~ kit.motor1.throttle = leftspeed
	# ~ kit.motor2.throttle = leftspeed
	# ~ kit.motor3.throttle = rightspeed
	# ~ kit.motor4.throttle = rightspeed
	# ~ Angle.append(t)
	# ~ Time.append(t)
	# ~ Target.append(t)
	# ~ Angle[t] = mirror_sensor_angle( sensor.euler[0] )
	# ~ Time[t] = t
	# ~ Target[t] = target
	# ~ t = t + 1
	# ~ time.sleep(.02)
	# ~ print(Distance(0))
	
kit.motor1.throttle = None
kit.motor2.throttle = None
kit.motor3.throttle = None
kit.motor4.throttle = None
print("finish turning")
print("target is : {}" .format(mirror_sensor_angle( sensor.euler[0] ) ))
graph(Time, Angle, Time, Target)
