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
  volts = (data * 3.3) / float(1024)
  volts = round(volts, 6) # Round off to 2 decimal places
  return volts

#The function below will convert voltage to distance from ultrasonic sensor
def Range(voltage):
    Range = voltage * (512/5)
    feet = round(Range, 2)
    meter = feet * .0348 # Convert to meter
    return meter
    
def Wall_Distance(voltage):
  distance = -0.467 * voltage + .987
  return distance
  
def Distance(channel):
  data = analogInput(channel)
  voltage = Volts(data)
  if (channel == 3 or channel == 5): # channel 3 and 4 are for ir sensor distance
    Distance = Wall_Distance(voltage)
    return Distance
  else:
    Distance = Range(voltage)
    return Distance
	

# ~ i = 0
# ~ while i < 100:
  # ~ print(Volts(analogInput(5)))
  # ~ print(Distance(5))
  # ~ print(Distance(3))
  # ~ sleep(.2)
  # ~ print()
  # ~ i = i + 1
  
