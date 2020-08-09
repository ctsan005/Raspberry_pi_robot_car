from adafruit_motorkit import MotorKit

#use to keep the angle from the sensor in the unit circle standard
def mirror_sensor_angle(angle):     
	return 360 - angle
    


#use to keep the angle in range, assume the angle not over 360 or -360
def convert_angle(error): 
    if error < -180:
        return (360 + error)
        
    elif error > 180:
        return (-360 + error) 
        
    else:
        return error
        
#use to return how to correct the motor speed speed to stay or reach some angle of the car to face
def pid(desired_value, actual_value, iteration_time , error_prior, integral_prior, kp,ki,kd):

    error = convert_angle(desired_value - actual_value)     #the difference between what the car need to face and the actual angle it is facing
    
    if(error < -300 or error > 300):        #if the error is too much, it mean the sensor has some gitch reading the angle. Use the previous data for now.
        error = error_prior
    
    if error > 2:                   #pid function is also use for turning, this help prevent the integral value affect the calculation too much
        integral = 0
    else:
        integral = integral_prior + error * iteration_time
    
    
    derivative = (error - error_prior) / iteration_time
    
    
    
    output = kp * error + ki * integral + kd * derivative
    error_prior = error
    integral_prior = integral
    
    return output, error_prior, integral_prior
    
def pid_wall(desired_value, actual_value, iteration_time , error_prior, integral_prior, kp,ki,kd):

    error = desired_value - actual_value
    
    
    # ~ if error > 2:                   #pid function is also use for turning, this help prevent the integral value affect the calculation too much
        # ~ integral = 0
    # ~ else:
        # ~ integral = integral_prior + error * iteration_time
        
    integral = integral_prior + error * iteration_time
    
    
    derivative = (error - error_prior) / iteration_time
    
    if(derivative > 1):
        derivative = 0
    
    output = kp * error + ki * integral + kd * derivative
    error_prior = error
    integral_prior = integral
    
    return output, error_prior, integral_prior
    




#Testing
# ~ a,b,c = pid(15,10,0.2,0,0, 0.02,0, 0)
# ~ print(a,b,c)
# ~ a,b,c = pid(10,5,0.2,b,c,1,0.01, 0.05)
# ~ print(a,b,c)
# ~ a,b,c = pid(10,17,0.2,b,c,1,0.01, 0.05)
# ~ print(a,b,c)


# ~ leftspeed = 1
# ~ rightspeed = 1
# ~ rightspeed, leftspeed, prev, sumError = pid(1, 10, rightspeed, leftspeed, 0, 0, .1, .005, .01)
# ~ print("Left Speed: {}     Right Speed: {}" .format(leftspeed,rightspeed))

# ~ rightspeed, leftspeed, prev, sumError = pid(1, 10, rightspeed, leftspeed, prev, sumError, .1, .005, .01)
# ~ print("Left Speed: {}     Right Speed: {}" .format(leftspeed,rightspeed))
