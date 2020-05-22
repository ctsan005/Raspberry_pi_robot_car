import time
import board
import busio
import adafruit_bno055

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


calibration()
