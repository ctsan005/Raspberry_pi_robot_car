# importing the required module 
import time
import board
import busio
import adafruit_bno055
import matplotlib.pyplot as plt 

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

Angle = []
Time = []

for x in range(10):
	Angle.append(x)
	Time.append(x)
	
	Angle[x] = sensor.euler[0]
	Time[x] = x
	time.sleep(1)

plt.plot(Time, Angle) 
plt.show() 
