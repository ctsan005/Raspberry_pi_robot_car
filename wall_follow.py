import time
import board
import busio
from subprocess import call
from motorFunction import pid,pid_wall
from Ultrasonic_Sensor import *
import adafruit_bno055
from adafruit_motorkit import MotorKit
import matplotlib.pyplot as plt


kit = MotorKit()
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

def graph(x1, y1, x2 ,y2):
    plt.plot(x1,y1, label = "line 1")
    plt.plot(x2,y2, label = "Target")
    plt.xlabel('Time')
    plt.ylabel('Distance')
    plt.legend()
    plt.show()


def wall_following(kp,ki,kd,rightspeed,leftspeed):

	start_time = time.time()

	target_distance_wall = 0.5      #need to adjust later, a variable for the target distance to the wall

	#init the car speed
	kit.motor1.throttle = leftspeed
	kit.motor2.throttle = leftspeed
	kit.motor3.throttle = rightspeed
	kit.motor4.throttle = rightspeed
	prev = 0
	sumError = 0


	#graph data
	t = 0
	Time = [] #Store Data for time
	wall_dis = [] #Store current angle at time t
	Target = [] #Store Target data as a reference


	#use for pid function to tell the iteration time for the pid funciton
	loop_time = time.time()
	time.sleep(.05)

	while( (time.time() - start_time) < 3): 
		if(Distance(0) < 0.6):
			print("hit wall")
			# ~ break
		
		output, prev, sumError = pid_wall(target_distance_wall, Distance(3), time.time() - loop_time,  prev, sumError, kp, ki, kd) #Call the pid function to obtain the change needed for the motor
		
		print(output,prev,sumError)
		
		loop_time = time.time()			#update the iteration time
		
		#update the motor speed
		rightspeed = max(0.3, min(1,rightspeed - output))
		leftspeed = max(0.3, min(1,leftspeed + output))
		
		print("The rightspeed and leftspeed is: {}, {}".format(rightspeed, leftspeed))
		kit.motor1.throttle = leftspeed
		kit.motor2.throttle = leftspeed
		kit.motor3.throttle = rightspeed
		kit.motor4.throttle = rightspeed

		#update the graph
		wall_dis.append(t)
		Time.append(t)
		Target.append(t)
		wall_dis[t] = Distance(3)
		Time[t] = time.time() - start_time
		Target[t] = target_distance_wall
		t = t + 1

		#sleep to prevent the pid run too fast
		time.sleep(.05)

		print(Distance(0))
		print(Distance(3))

	#stop the car to prevent the car to hit the wall
	kit.motor1.throttle = None
	kit.motor2.throttle = None
	kit.motor3.throttle = None
	kit.motor4.throttle = None
	
	graph(Time, wall_dis, Time, Target)

while(1):
	kit.motor1.throttle = None			#Goof kp ki kd value are 0.01, 0.005, 0.01
	kit.motor2.throttle = None
	kit.motor3.throttle = None
	kit.motor4.throttle = None
	print(Distance(0))
	print(Distance(3))
	kp = input("please enter kp value: ")
	ki = input("please enter ki value: ")
	kd = input("please enter kd value: ")
	# ~ #wall_following(float(kp),float(ki),float(kd),1,1)
	kit.motor1.throttle = 1			#Goof kp ki kd value are 0.01, 0.005, 0.01
	kit.motor2.throttle = 1
	kit.motor3.throttle = 1
	kit.motor4.throttle = 1	
	time.sleep(0.1)
	wall_following(float(kp),float(ki),float(kd),1,1)			#kp = 0.15, ki = 0.036, kd = 0.33	
	
# ~ while(1):
	# ~ print(Distance(3))
