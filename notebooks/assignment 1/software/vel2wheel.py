import numpy as np
def vel2wheel(v, omega, wheel_dist, wheel_rad):
    
    gain = 1
    trim = 0
    
    # Maximal speed
    if v > 0.5:
        v = 0.5
    elif v < -0.5:
        v = -0.5
    
    ##### Fill the code here:
    right_rate =  v +  (0.5 * wheel_dist * omega)
    left_rate = v -  (0.5 * wheel_dist * omega)
    right_rate /=(wheel_rad * np.pi * 2)
    left_rate  /=(wheel_rad * np.pi * 2)
    
    ####


    
    
    return left_rate, right_rate