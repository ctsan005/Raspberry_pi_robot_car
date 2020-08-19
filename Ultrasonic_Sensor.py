# Importing modules
import spidev # To communicate with SPI devices
from time import sleep  # To add delay
import statistics 

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
    return round(meter, 2)
    
def Wall_Distance(voltage):
  distance = -0.467 * voltage + .987
  return round(distance, 2)
  
def Distance(channel):
  # ~ data = analogInput(channel)
  # ~ voltage = Volts(data)
  # ~ if (channel == 3 or channel == 5): # channel 3 and 4 are for ir sensor distance
    # ~ Distance = Wall_Distance(voltage)
    # ~ return Distance
  # ~ else:
    # ~ Distance = Range(voltage)
    # ~ return Distance
    
    
  voltage = []
  voltage.append(Volts(analogInput(channel)))  
  # ~ sleep(0.06)
  # ~ voltage.append(Volts(analogInput(channel))) 
  # ~ sleep(0.06)
  # ~ voltage.append(Volts(analogInput(channel))) 
  # ~ sleep(0.06)
  # ~ voltage.append(Volts(analogInput(channel))) 
  # ~ sleep(0.06)
  # ~ voltage.append(Volts(analogInput(channel))) 

  
  median_volt = statistics.median(voltage)
  
  if (channel == 3 or channel == 5): # channel 3 and 4 are for ir sensor distance
    Distance = Wall_Distance(median_volt)
    return Distance
  else:
    Distance = Range(median_volt)
    return Distance
	
def Distance2(channel, last_val):
  # ~ data = analogInput(channel)
  # ~ voltage = Volts(data)
  # ~ if (channel == 3 or channel == 5): # channel 3 and 4 are for ir sensor distance
    # ~ Distance = Wall_Distance(voltage)
    # ~ return Distance
  # ~ else:
    # ~ Distance = Range(voltage)
    # ~ return Distance
    
    
  voltage = Volts(analogInput(channel))
  # ~ sleep(0.06)
  # ~ voltage.append(Volts(analogInput(channel))) 
  # ~ sleep(0.06)
  # ~ voltage.append(Volts(analogInput(channel))) 
  # ~ sleep(0.06)
  # ~ voltage.append(Volts(analogInput(channel))) 
  # ~ sleep(0.06)
  # ~ voltage.append(Volts(analogInput(channel))) 

  
  if (channel == 3 or channel == 5): # channel 3 and 4 are for ir sensor distance
    Distance = Wall_Distance(voltage)
    return Distance * 0.2 + last_val * 0.8
  else:
    Distance = Range(voltage)
    return Distance * 0.2 + last_val * 0.8
    
    
i = 0
last_val = 0
while i < 100:
  print(Volts(analogInput(0)))
  print(analogInput(0))
  last_val = Distance2(0,last_val)
  print(last_val)
  sleep(.06)
  print()
  i = i + 1
  
