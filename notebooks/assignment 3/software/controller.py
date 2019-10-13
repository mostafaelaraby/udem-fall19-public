import numpy as np
import math

class Controller():
    def __init__(self):
        self.gain = 2.0
        pass

    def angle_control_commands(self, dist, angle):
        # Return the angular velocity in order to control the Duckiebot so that it follows the lane.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        # Outputs:
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        omega = 0. 
        
        #######
        v=0.5
        k_theta = 12
    
        k_dist = ((1 * k_theta**2)  / (4*v))   
        
        theta_threshold =  math.pi/6
        
        d_threshold = abs(theta_threshold  *  (k_theta * 1.0 / k_dist))
        
        if dist>= -1 * d_threshold and dist<=d_threshold:
            pass
        elif dist< d_threshold:
            dist =  -1 * d_threshold
        elif dist>d_threshold:
            dist = d_threshold
        
        if abs(angle) > theta_threshold:
            sign = lambda x: (1, -1)[x < 0]
            angle = theta_threshold * sign(angle)

        omega =  1 * ((k_theta * angle) + (k_dist * dist) ) 
        #######

        return  omega

    def pure_pursuit(self, env, pos, angle, follow_dist=0.25):
        # Return the angular velocity in order to control the Duckiebot using a pure pursuit algorithm.
        # Parameters:
        #     env: Duckietown simulator
        #     pos: global position of the Duckiebot
        #     angle: global angle of the Duckiebot
        # Outputs:
        #     v: linear veloicy in m/s.
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        
        closest_curve_point = env.unwrapped.closest_curve_point
        
        # Find the curve point closest to the agent, and the tangent at that point
        closest_point, closest_tangent = closest_curve_point(pos, angle)
        iterations = 0
        
        lookup_distance = follow_dist
        multiplier = 0.5
        curve_point = None
        
        while iterations < 10:            
            ########
            #
            #TODO 1: Modify follow_point so that it is a function of closest_point, closest_tangent, and lookup_distance
            #
            ########
            follow_point = closest_point + closest_tangent * lookup_distance#[closest_point[i] +  lookup_distance * closest_tangent[i] for i in range(3)]
            
            curve_point, _ = closest_curve_point(follow_point, angle)

            # If we have a valid point on the curve, stop
            if curve_point is not None:
                break

            iterations += 1
            lookup_distance *= multiplier
        ########
        #
        #TODO 2: Modify omega
        #
        ########
        v = 0.5

        #normalized vector
        point_vec = curve_point - env.cur_pos
        point_vec /= np.linalg.norm(point_vec)

        # move the vector by the bot's angle
        right_vec =  np.array([ math.sin(angle), 0, math.cos(angle)]) 
        dot = np.dot(right_vec, point_vec)
        # move the bot in that direction
        gain = 10
        omega = -1 * gain * dot

        return v, omega