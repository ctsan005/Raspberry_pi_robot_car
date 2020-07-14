import math
import numpy as np
# ~ L = 1 # Distance between wheels
# ~ VL = 20 # Velocity of left wheel
# ~ VR = 15 # Velocity of right wheel
# ~ dt = .1
# ~ #initial position
# ~ x = 0
# ~ y = 0
# ~ radian = 0
def FWDKIN(x, y, radian, dt, L, VL, VR):    
    #need to update VL VR every ~ .1s
    base = VR - VL # wheel spinning same velocity. going straight
    
    if base == 0: #Straight case
        ans = [[x + VR*math.cos(radian)*dt],
              [y + VR*math.sin(radian)*dt],
              [radian]]
#         print(ans)
    
    else: #Turning case
        R = (L/2)*((VL + VR)/(VR - VL)) # Calculate radius of turn
        w = ((VR - VL) / L) # Calcualte rate of rotation
        ICCx = (x - R*math.sin(radian))
        ICCy = (y + R*math.cos(radian))

        A = [[math.cos(w*dt), -1*math.sin(w*dt), 0],
             [math.sin(w*dt), math.cos(w*dt), 0],
             [0, 0, 1]]
        B = [[x - ICCx],
            [y - ICCy],
            [radian]]
        C =  [[ICCx],
             [ICCy],
             [w*dt]]
        ans = np.dot(A,B)
        ans = ans + C 
#         print(ans)
    return ans 
# ~ print(FWDKIN(0, 0, 0, .1, 1, 20, 15))
