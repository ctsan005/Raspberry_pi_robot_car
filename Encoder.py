import serial
from time import sleep
from adafruit_motorkit import MotorKit
import time

ser = serial.Serial(
    port= '/dev/serial0',
    baudrate = 9600, 
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

kit = MotorKit()
def control_speed(left, right):
	kit.motor1.throttle = left
	kit.motor2.throttle = left
	kit.motor3.throttle = right
	kit.motor4.throttle = right

# ~ while True:
	# ~ recieved_data = ser.read()
	# ~ sleep(.01)
	# ~ data_left = ser.inWaiting()
	# ~ recieved_data += ser.read(data_left)
	# ~ print (recieved_data)
	
#use for reading the encoder information, return a 16 bits data, left 8 bits is one motor information and right 8 bits is another motor information
def readencoder():
	recieved_data = ser.read()
	data_left = ser.inWaiting()
	recieved_data += ser.read(data_left)
	return recieved_data
	
# ~ print(readencoder())
# ~ print(len(readencoder()))

#maybe me setup a warning to check can the readencoder return two data point back
def wheelspeed():
	temp = readencoder()
	count = 0
	# ~ print(temp)
	# ~ print(len(temp))
	# wheel diameter = .12m, Circumference = .377m, distance between left and right wheel = 0.229m
	# ~ # 1 revolution is 2249 counts
	# ~ print(temp)
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
	# ~ print(frequencyL)
	# ~ print(frequencyR)
	VL = (frequencyL/2249) * .377 * 4
	VR = (frequencyR/2249) * .377 * 4		# multiply by 4 to fix the velocity error, need to check later
	return VL , VR

# ~ control_speed(1,1)
# ~ i = 0
# ~ while i < 10:
	# ~ print(wheelspeed())
	# ~ i = i + 1
	# ~ time.sleep(.2)
# ~ print()
# ~ control_speed(0.5,0.5)
# ~ i = 0
# ~ while i < 10:
	# ~ print(wheelspeed())
	# ~ i = i + 1
	# ~ time.sleep(.2)
# ~ control_speed(None,None)

# ~ control_speed(None, None)
# ~ input("press enter to start")
# ~ control_speed(1,1)
# ~ time.sleep(3)
# ~ print(wheelspeed())
# ~ control_speed(None,None)
	

# ~ temp = readencoder()
# ~ print(temp)
# ~ print(type(temp))

# This is how to read the bytes format of the information


# ~ print(temp[0])		#read the left 8 bits data and print a decimal number
# ~ print(temp[1])		#read the right 8 bits data and print a decimal number
# ~ print (recieved_data)
