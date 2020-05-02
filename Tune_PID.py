import time
import board
import busio
from subprocess import call
from motorFunction import *
import adafruit_bno055
from adafruit_motorkit import MotorKit
import matplotlib.pyplot as plt 

kit = MotorKit()
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)
# ~ calibrated = 0
# ~ while calibrated != 1 :
    # ~ sys, gyro, accel, mag =  sensor.calibration_status
    # ~ print("calibration: sys:{} gyro:{} accel:{} mag:{}".format(sys,gyro, accel, mag))
    # ~ if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
        # ~ print("calibration done")
        # ~ calibrated = 1
    # ~ time.sleep(0.5)
    
# ~ print("IMU calibrated")
# ~ print("press enter to set target orientation")
# ~ input()

def graph(x1, y1, x2 ,y2):
    plt.plot(x1,y1, label = "line 1")
    plt.plot(x2,y2, label = "Target")
    plt.xlabel('Time')
    plt.ylabel('Angle')
    plt.show()
    
def graph2(x1, y1, x2 ,y2):
    plt.plot(x1,y1, label = "line 1")
    plt.plot(x2,y2, label = "Target")
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.show()
    
def calibration():
	i2c = busio.I2C(board.SCL, board.SDA)
	sensor = adafruit_bno055.BNO055(i2c)
	calibrated = 0
	while calibrated != 1 :
		sys, gyro, accel, mag =  sensor.calibration_status
		print("calibration: sys:{} gyro:{} accel:{} mag:{}".format(sys,gyro, accel, mag))
		if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
			print("calibration done")
			calibrated = 1
		time.sleep(0.5)

	print("Calibrated")
	print("press enter to continue")
	input()
	return sensor

def straight_car(sensor, Kp, Ki, Kd):
    Time = [] #Store Data for time
    Angle = [] #Store current angle at time t
    Target = [] #Store Target data as a reference
    leftspeedlist = [] #store the left speed
    rightspeedlist = [] #store the right speed
    t = 0 # Initialize time t=0
    print("Kp is {} Ki is {} Kd is {}" .format(Kp, Ki, Kd))
    kit = MotorKit()
    target = sensor.euler[0]
    print("Target Angle: {}" .format(sensor.euler[0]))
    leftspeed = 1
    rightspeed = 1
    prev = 0
    sumError = 0
    kit.motor1.throttle = leftspeed
    kit.motor2.throttle = leftspeed
    kit.motor3.throttle = rightspeed
    kit.motor4.throttle = rightspeed
    count = 0
    sleeptime = 0.02
    start_time = time.time()
    
    while True:
        try:
            rightspeed, leftspeed, prev, sumError = pid(target, sensor, rightspeed, leftspeed, prev, sumError, Kp, Ki, Kd)
            kit.motor1.throttle = leftspeed
            kit.motor2.throttle = leftspeed
            kit.motor3.throttle = rightspeed
            kit.motor4.throttle = rightspeed
            print("right speed: {}  left speed: {} preverror: {} sumError: {}" .format(round(rightspeed,3), round(leftspeed,3), prev, sumError))
            print("current Angle: {}" .format(sensor.euler[0]))
            Angle.append(t)
            Time.append(t)
            Target.append(t)
            Angle[t] = sensor.euler[0]
            Time[t] = t
            Target[t] = target
            leftspeedlist.append(leftspeed)
            rightspeedlist.append(rightspeed)
            t = t + 1
            time.sleep(sleeptime) #50 Hz
            count = count + 1
            # ~ if(count > 0.3385/sleeptime):        #not exact 2 sec time, other reason lead to longer time, need adjust
                # ~ print(time.time()-start_time)
                # ~ break
        except KeyboardInterrupt:
            kit.motor1.throttle = None
            kit.motor2.throttle = None
            kit.motor3.throttle = None
            kit.motor4.throttle = None
            graph(Time, Angle, Time, Target)
            graph2(Time, leftspeedlist, Time, rightspeedlist)
            print("Receieve keyboard Interrupt and stop the straight_car function")
            break
            
# ~ while True:
    # ~ sensor = calibration()
    # ~ sensor = adafruit_bno055.BNO055(i2c) 
    # ~ while True:
        # ~ kit.motor1.throttle = None
        # ~ kit.motor2.throttle = None
        # ~ kit.motor3.throttle = None
        # ~ kit.motor4.throttle = None
        # ~ input("Press enter to start straight car function and ctr-c to stop it")
        # ~ Kp = input("what is Kp: ") #.5
        # ~ Ki = input("what is Ki: ") #.01
        # ~ Kd = input("what is Kd: ") #.2
        # ~ straight_car(sensor, float(Kp), float(Ki), float(Kd))

