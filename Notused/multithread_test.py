# Python program to illustrate the concept 
# of threading 
import threading 
import os 
from time import sleep
# ~ from Straight_Car import *
from Velocity_Distance_Calculation import *

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

def straight_car(sensor):
    
    kit = MotorKit()
    target = sensor.euler[0]
    leftspeed = 1.0
    rightspeed = 1.0
    prev = 0
    sumError = 0
    kit.motor1.throttle = leftspeed
    kit.motor2.throttle = rightspeed
    while True:
        try:
            rightspeed, leftspeed, prev, sumError = pid(target, sensor, rightspeed, leftspeed, prev, sumError, .1, .005, .01)
            kit.motor1.throttle = leftspeed
            kit.motor2.throttle = rightspeed
            print("right speed: {}  left speed: {} preverror: {} sumError: {}" .format(round(rightspeed,3), round(leftspeed,3), prev, sumError))
            time.sleep(1)
        except KeyboardInterrupt:
            print("Receieve keyboard Interrupt and stop the straight_car function")
            break

finish = False
  
def task1(item): 
    # ~ print("Task 1 assigned to thread: {}".format(item)) 
    # ~ print("ID of process running task 1: {}".format(os.getpid())) 
   while(finish == False):
      print("Not finish")
      sleep(1)
   print("finish")
       

  
def task2(): 
    print("Task 2 assigned to thread: {}".format(threading.current_thread().name)) 
    print("ID of process running task 2: {}".format(os.getpid())) 
    
def task3(item):
   
   input("press enter if you want the pid to finish")
   finish = True
   print("finish is true")
  
if __name__ == "__main__": 
   
   # ~ sensor = calibration()
   t1 = threading.Thread(target=straight_car,args=(sensor,) ) 
   # ~ t2 = threading.Thread(target=velocity_calculation, args=(sensor,))
   # ~ t2.start()  
   while True:
      
      # ~ input("press enter to start stright_car function")
    # print ID of current process 
    # ~ print("ID of process running main program: {}".format(os.getpid())) 
  
    # print name of main thread 
    # ~ print("Main thread name: {}".format(threading.current_thread().name)) 
  
    # creating threads 
    # ~ a = "hello"
    

    
    #testing purpose
    # ~ t1 = threading.Thread(target=task1,args=(finish,) ) 
    # ~ t2 = threading.Thread(target=task3, args=(finish,))
      
  
    # starting threads 
      t1.start()
      print("test1") 
    
  
    # wait until all threads finish 
      while t1.isAlive():
         t1.join(5)
      # ~ t1.join() 
      print("after join")
    # ~ t2.join() 

