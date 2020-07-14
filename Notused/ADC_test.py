# Importing modules
import spidev # To communicate with SPI devices
from time import sleep  # To add delay

# Start SPI connection
spi = spidev.SpiDev() # Created an object
spi.open(0,0) 

# Read MCP3008 data
def analogInput(channel):
  spi.max_speed_hz = 1350000
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data

# Below function will convert data to voltage
def Volts(data):
  volts = (data * 3.3) / float(1023)
  volts = round(volts, 6) # Round off to 2 decimal places
  return volts

#The function below will convert voltage to distance from ultrasonic sensor
def Distance(voltage):
    distance = voltage * (512/5)
    distance = round(distance, 2)
    return distance

while True:
    adcraw = analogInput(0)
    voltage = Volts(adcraw)
    distance = Distance(voltage)
    print("adcraw: {}     Voltage : {}     Distance: {} in.".format(adcraw,voltage,distance))
    print("")
    sleep(.06)
