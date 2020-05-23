import time
import board
import busio
import adafruit_bno055
 
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)
calibrated = 0
# ~ while calibrated != 1 :
    # ~ sys, gyro, accel, mag =  sensor.calibration_status
    # ~ print("calibration: sys:{} gyro:{} accel:{} mag:{}".format(sys,gyro, accel, mag))
    # ~ if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
        # ~ print("calibration done")
        # ~ calibrated = 1
    # ~ time.sleep(0.5)
while True:
    print("Temperature: {} degrees C".format(sensor.temperature))
    # ~ print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    # ~ print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    # ~ print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    # ~ print("Euler angle: {}".format(sensor.euler))
    # ~ print("Quaternion: {}".format(sensor.quaternion))
    # ~ print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    # ~ print("Gravity (m/s^2): {}".format(sensor.gravity))
    print()
 
    # ~ time.sleep(1)
# ~ while True:
    # ~ current_angle = 360 - sensor.euler[0]
    # ~ print (current_angle)
