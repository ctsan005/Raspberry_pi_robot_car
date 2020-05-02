import time
import board
import busio
from subprocess import call
from motorFunction import *
import adafruit_bno055
from adafruit_motorkit import MotorKit

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
    sleeptime = 0.02
    startime = time.time()
    
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
            if(time.time() - startime > 2):
                break
        except KeyboardInterrupt:
            kit.motor1.throttle = None
            kit.motor2.throttle = None
            kit.motor3.throttle = None
            kit.motor4.throttle = None
            graph(Time, Angle, Time, Target)
            graph2(Time, leftspeedlist, Time, rightspeedlist)
            print("Receieve keyboard Interrupt and stop the straight_car function")
            break
    
def turning(sensor, target):        #need to precalculate the target angle using euler angle, this help prevent re-reading the euler angle and get a different angle
    # to control the left and right motor
    kit = MotorKit()
    
    # the target angle we want to reach
    leftspeed = 1.0
    rightspeed = 1.0
    prev = 0
    sumError = 0
    kit.motor1.throttle = leftspeed
    kit.motor2.throttle = leftspeed
    kit.motor3.throttle = rightspeed
    kit.motor4.throttle = rightspeed
    while True:
        try:
            rightspeed, leftspeed, prev, sumError = pid(target, sensor, rightspeed, leftspeed, prev, sumError, .5, .01, .2)
            kit.motor1.throttle = leftspeed
            kit.motor2.throttle = leftspeed
            kit.motor3.throttle = rightspeed
            kit.motor4.throttle = rightspeed
            print("right speed: {}  left speed: {} preverror: {} sumError: {}" .format(round(rightspeed,3), round(leftspeed,3), prev, sumError))
            time.sleep(.02)
        except KeyboardInterrupt:
            print("Receieve keyboard Interrupt and stop the straight_car function")
            break

def rotate_in_place(sensor, angle):
    # to control the left and right motor
    kit = MotorKit()
    
    target = sensor.euler[0] + angle
    if target > 360:
        target = target - 360
    
    if target > 180:
        leftspeed = 0.5
        rightspeed = -0.5
        
    else:
        leftspeed = -0.5
        rightspeed = 0.5
        
    kit.motor1.throttle = leftspeed
    kit.motor2.throttle = rightspeed
    
    
    while sensor.euler[0] != target: pass
        
    leftspeed = 1
    rightspeed = 1
    kit.motor1.throttle = leftspeed
    kit.motor2.throttle = rightspeed
