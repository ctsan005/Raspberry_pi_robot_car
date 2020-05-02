import time
import serial

ser = serial.Serial(
        port='/dev/serial0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=.5
)

while 1:
    print("working")
    x=ser.read()
    print("done")
    print(x)