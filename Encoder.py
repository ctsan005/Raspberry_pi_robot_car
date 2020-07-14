import serial
from time import sleep
from adafruit_motorkit import MotorKit
import time

#Define UART 
ser = serial.Serial(
    port= '/dev/serial0',
    baudrate = 9600, 
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

#Returns serial data from Nucleo, return a 16 bits data, left 8 bits is one motor information and right 8 bits is another motor information
def readencoder():
	recieved_data = ser.read()
	data_left = ser.inWaiting()
	recieved_data += ser.read(data_left)
	return recieved_data

#maybe me setup a warning to check can the readencoder return two data point back
def wheelspeed():
	temp = readencoder()
	count = 0
	# wheel diameter = .12m, Circumference = .377m, distance between left and right wheel = 0.229m
	# ~ # 1 revolution is 2249 count
    
    #Clears buffer. Nucleo constantly spits data out, so need to clear buffer before reading next data
	while(len(temp) != 2):
		if count < 10:
			temp = readencoder()
			# ~ print(temp)
			count=count + 1
		else:
			raise ValueError ('encoder value is wrong')
		# ~ print("The reading for encoder is wrong, take a look!!")
		# ~ raise ValueError ('encoder value is wrong')
            
	frequencyL = temp[0] * 10 #pin D4 on nucleo left wheel encoder
	frequencyR = temp[1] * 10 #pin D7 on nucleo right wheel encoder
	VL = (frequencyL/2249) * .377 * 4
	VR = (frequencyR/2249) * .377 * 4		# multiply by 4 to fix the velocity error, need to check later
	return VL , VR
