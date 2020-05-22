import serial
from time import sleep

ser = serial.Serial(
    port= '/dev/serial0',
    baudrate = 9600, 
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
	
# ~ print(readencoder())
# ~ print (recieved_data)
