# importing the required module 
import time
import board
import busio
import adafruit_bno055
import matplotlib.pyplot as plt 

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)
  
# ~ # x axis values 
# ~ x = [1,2,3] 
# ~ # corresponding y axis values 
# ~ y = [2,4,1] 
  
# ~ # plotting the points  
# ~ plt.plot(x, y) 
  
# ~ # naming the x axis 
# ~ plt.xlabel('x - axis') 
# ~ # naming the y axis 
# ~ plt.ylabel('y - axis') 
  
# ~ # giving a title to my graph 
# ~ plt.title('My first graph!') 
  
# ~ # function to show the plot 
# ~ plt.show() 

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
