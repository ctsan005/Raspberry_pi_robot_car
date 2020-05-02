import serial
from time import sleep
ser= serial.Serial("/dev/serial0", 9600)

while True:
    print("before receieved data")
    recieved_data = ser.read()
#     data_left = ser.inWaiting()
#     recieved_data += ser.read(data_left)
#     print("after receieved data")
#     print(type(recieved_data))
#     if 
    print(recieved_data.hex())
    print (recieved_data)
#     sleep(1)
    