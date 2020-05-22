import serial
from time import sleep

ser = serial.Serial(
    port= '/dev/serial0',
    baudrate = 115200, 
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
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
	
print(readencoder())
print(len(readencoder()))

#maybe me setup a warning to check can the readencoder return two data point back
def wheelspeed():
	temp = readencoder()
	# wheel diameter = .12m, Circumference = .377m, distance between left and right wheel = 0.229m
	# ~ # 1 revolution is 2249 counts
	# ~ print(temp)
	frequencyL = temp[0] * 10
	frequencyR = temp[1] * 10
	VL = (frequencyL/2249) * .377
	VR = (frequencyR/2249) * .377
	return VL , VR
	
# ~ print(wheelspeed())
	

# ~ temp = readencoder()
# ~ print(temp)
# ~ print(type(temp))

# This is how to read the bytes format of the information


# ~ print(temp[0])		#read the left 8 bits data and print a decimal number
# ~ print(temp[1])		#read the right 8 bits data and print a decimal number
# ~ print (recieved_data)
