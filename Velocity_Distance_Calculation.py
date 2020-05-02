import time
import board
import busio
import adafruit_bno055
from time import sleep

def calibration():
	i2c = busio.I2C(board.SCL, board.SDA)
	sensor = adafruit_bno055.BNO055(i2c)
	calibrated = 0
	while calibrated != 1 :
		sys, gyro, accel, mag =  sensor.calibration_status
		print("calibration: sys:{} gyro:{} accel:{} mag:{}".format(sys,gyro, accel, mag))
		if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
			print("calibration done")
			calibrated = 1
		time.sleep(0.5)

	print("Calibrated")
	print("press enter to continue")
	input()
	return sensor

# ~ accel_x = sensor.linear_acceleration[0]
# ~ accel_y = sensor.linear_acceleration[1]

def velocity_calculation(sensor):
	#Calculate velocity V = V0 + at and distance traveled dx = Vo*t +1/2 * a *t^2
	Velocity = 0
	d_traveled = 0
	# ~ sampletime= .0125 #80 hz
	sampletime = 3 #testing purpose, need to change back later
	while True:
		accel_x = sensor.linear_acceleration[0]
		dx = Velocity * sampletime + 1/2 * accel_x * (sampletime**2) 
		Velocity = Velocity + accel_x * sampletime
		
		d_traveled += dx
		print("Velocity: {} m/s     Distance traveled: {} m" .format(Velocity, d_traveled))
		sleep(sampletime)
