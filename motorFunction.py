from adafruit_motorkit import MotorKit
#use to control the right motor, 

def mirror_sensor_angle(angle):
	return 360 - angle
    
def rightmotorSet(number):
    if number > 1.0:
        number = 1.0
    if number < 0:
        number = 0
    kit.motor1.throttle = number
    kit.motor2.torottle = number
    
#use to control the left motor   
def leftmotorSet(number):
    if number > 1.0:
        number = 1.0
    if number < 0:
        number = 0
    kit.motor3.throttle = number
    kit.motor4.torottle = number
    
# ~ def convert_error(error):   #10 degree is very bad, so we convert 10 degree to 0.1 motor speed
    # ~ return error * 0.01
    
def convert_error(error, ratio = 0.01): #potential problem if our error is greater than 180. This assumes our error is not greater than 180
    if error < -180:
        return (360 + error) * ratio #number to define linear relationship between error and motor speed
        
    elif error > 180:
        return (-360 + error) * ratio 
        
    else:
        return error * ratio
    
    
def pid(target, sensor,  rightmotorSpeed, leftmotorSpeed, prevError, sumError, Kp , Ki , Kd):       #need to change the correction to also add speed to motor
    current_angle = mirror_sensor_angle( sensor.euler[0] ) #flip to unit circle conventions
    if(current_angle < -400):
        error = prevError       #handle error case with -1700 angle
        print("error happen with angle")
    else:    
        #----------------------------------------------------------------------------------------------------------------------
        # ~ error = target - current_angle #probably need to flip to change axis conventions
        error = current_angle - target      #The correct version? need test physically 
        #----------------------------------------------------------------------------------------------------------------------
    
    convert_speed = convert_error(error) #convert the error to speed that need to modify on the mootor
    if convert_speed < 0: # turn left
        # ~ print("turning left")
        leftmotorSpeed = leftmotorSpeed - (convert_speed * Kp * -1) - ((convert_speed - prevError) * Kd) - (sumError * Ki)
        leftmotorSpeed = max(0.4,min(1,leftmotorSpeed))
        
        rightmotorSpeed = rightmotorSpeed + (convert_speed * Kp * -1) + ((convert_speed - prevError) * Kd) + (sumError * Ki)
        rightmotorSpeed = max(0.4,min(1,rightmotorSpeed))
        
        prevError = convert_speed
        sumError += convert_speed
    elif convert_speed > 0: #turn right
        # ~ print("turning right")
        rightmotorSpeed = rightmotorSpeed - (convert_speed * Kp) - ((convert_speed - prevError) * Kd) - (sumError * Ki)
        rightmotorSpeed = max(0.4,min(1,rightmotorSpeed))
        
        leftmotorSpeed = leftmotorSpeed + (convert_speed *Kp) + ((convert_speed - prevError) * Kd) + (sumError * Ki)
        leftmotorSpeed = max(0.4,min(1,leftmotorSpeed))
        
        prevError = convert_speed
        sumError += convert_speed
    return rightmotorSpeed, leftmotorSpeed, prevError, sumError

# ~ leftspeed = 1
# ~ rightspeed = 1
# ~ rightspeed, leftspeed, prev, sumError = pid(1, 10, rightspeed, leftspeed, 0, 0, .1, .005, .01)
# ~ print("Left Speed: {}     Right Speed: {}" .format(leftspeed,rightspeed))

# ~ rightspeed, leftspeed, prev, sumError = pid(1, 10, rightspeed, leftspeed, prev, sumError, .1, .005, .01)
# ~ print("Left Speed: {}     Right Speed: {}" .format(leftspeed,rightspeed))
