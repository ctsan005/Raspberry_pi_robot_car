import time
from adafruit_motorkit import MotorKit
 
kit = MotorKit()
 
kit.motor1.throttle = 1.0
kit.motor2.throttle = 1.0
kit.motor3.throttle = 1.0
kit.motor4.throttle = 1.0
time.sleep(2)
kit.motor1.throttle = 0
kit.motor2.throttle = 0
kit.motor3.throttle = 0
kit.motor4.throttle = 0
print("WORKS")

