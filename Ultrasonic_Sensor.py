# Importing modules
import spidev # To communicate with SPI devices
from time import sleep  # To add delay
from statistics import median
import numpy

#keep track the last distance value for each sensor, need to call the init_distance function to get the best result before run
distance = [0,0,0,0,0]


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

def init_distance():
  #get the array from global
  global distance

  #create a 2d array to store the result for the collection of data
  sensor_val = numpy.zeros((5, 5))
  
  #read the value 5 times for each sensor
  for a in range(5):
    sensor_val[0][a] = Range(Volts(analogInput(0)))
    sensor_val[1][a] = Range(Volts(analogInput(1)))
    sensor_val[2][a] = Range(Volts(analogInput(2)))
    sensor_val[3][a] = Wall_Distance(Volts(analogInput(3)))
    sensor_val[4][a] = Wall_Distance(Volts(analogInput(4)))
    sleep(0.11)

  #get the median for each sensor reading
  for a in range(5):
    distance[a] = median(sensor_val[a])

  print("Finish init the sensor distance, ready to run")
  print("The init distance for all sensor are: {}".format(distance))
  return True

  



  
def Distance(channel):
  # this variable is for how many percent to trust the new reading for the sensor value
  percent = 0.99
  global distance
  data = analogInput(channel)
  voltage = Volts(data)
  if (channel == 3 or channel == 4): # channel 3 and 4 are for ir sensor distance
    Distance = Wall_Distance(voltage)
  else:
    Distance = Range(voltage)
  
  distance[channel] = Distance * percent + distance[channel] * (1-percent)
  return round(distance[channel],2)
  
# ~ def Distance_old(channel):
  # ~ global distance
  # ~ data = analogInput(channel)
  # ~ voltage = Volts(data)
  # ~ if (channel == 3 or channel == 4): # channel 3 and 4 are for ir sensor distance
    # ~ Distance = Wall_Distance(voltage)
  # ~ else:
    # ~ Distance = Range(voltage)

  # ~ return round(Distance,2)
	

# ~ i = 0
# ~ while i < 100:
  # ~ print(Volts(analogInput(5)))
  # ~ print(Distance(5))
  # ~ print(Distance(3))
  # ~ sleep(.2)
  # ~ print()
  # ~ i = i + 1
  
