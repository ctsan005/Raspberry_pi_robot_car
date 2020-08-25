import time
import board
import busio
from subprocess import call
from motorFunction import *
from motorFunction import pid
from Ultrasonic_Sensor import *
import adafruit_bno055
from adafruit_motorkit import MotorKit

kit = MotorKit()
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

# ~ sensor = calibration() #calibrate sensor
sensor = adafruit_bno055.BNO055(i2c) 

#use for testing the pid function with two parts, first going straight and then make a turn by setting the target angle slightly different. It will generate a graph for the angle the car that it faced vs the target of the angle that it should face.
#Distance 0 = Middle
#Distance 1 = Left
#Distance 2 = Right
def test_turn(speed):
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


    #use for pid function to tell the iteration time for the pid funciton
    loop_time = time.time()

	#continue to go straight until 0.5 sec or there is a obstacle that is closer than 0.6 distance from the car
    while( Distance(0) > 1): 

        
        output, prev, sumError = pid(target, mirror_sensor_angle( sensor.euler[0] ), time.time() - loop_time,  prev, sumError, 0.05, 0.012, 0.011)		#call the pid function to calculate the speed change required
        
        loop_time = time.time()			#update the iteration time for the pid

        #update the speed of the left and right motor
        rightspeed = max(0.3, min(1,rightspeed + output))
        leftspeed = max(0.3, min(1,leftspeed - output))

        print("The rightspeed and leftspeed is: {}, {}".format(rightspeed, leftspeed))

        kit.motor1.throttle = leftspeed
        kit.motor2.throttle = leftspeed
        kit.motor3.throttle = rightspeed
        kit.motor4.throttle = rightspeed

        #sleep to prevent the pid function run too fast
        time.sleep(.02)

    print("finish straight")

    print("start turning")

    #update the motor speed
    rightspeed = 1 - speed
    leftspeed = 1
    
    print("The rightspeed and leftspeed is: {}, {}".format(rightspeed, leftspeed))
    kit.motor1.throttle = leftspeed
    kit.motor2.throttle = leftspeed
    kit.motor3.throttle = rightspeed
    kit.motor4.throttle = rightspeed


    #sleep to prevent the pid run too fast
    time.sleep(3)


    #stop the car to prevent the car to hit the wall
    kit.motor1.throttle = None
    kit.motor2.throttle = None
    kit.motor3.throttle = None
    kit.motor4.throttle = None

	

#testing the function
while True:
    kit.motor1.throttle = None			#Goof kp ki kd value are 0.01, 0.005, 0.01
    kit.motor2.throttle = None
    kit.motor3.throttle = None
    kit.motor4.throttle = None
    print(Distance(0))
    turn = input("number of speed of turn: ")
    test_turn(float(turn))
