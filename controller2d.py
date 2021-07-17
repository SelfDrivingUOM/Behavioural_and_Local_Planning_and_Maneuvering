#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""
import glob
import os
import sys
import time
from os_carla import *
# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
if WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Carla0.99/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass

elif YASINTHA_WINDOWS:
    try:
        sys.path.append(glob.glob('C:/Users/4Axis/Desktop/Project/Carla/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass
    
elif GERSHOM_WINDOWS:
    try:
        sys.path.append(glob.glob('D:/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg' )[0])
    except IndexError:
        pass
elif SAUMYA_UBUNTU:
    try:
        sys.path.append(glob.glob('/home/pq-saumya/Documents/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass
    
else:
    try:
        sys.path.append(glob.glob('/home/selfdriving/carla-precompiled/CARLA_0.9.9/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg' )[0])
    except IndexError:
        pass

import cutils
import numpy as np
import carla

#from mpc import MPC
from testnmpc import NMPC

np.set_printoptions(linewidth=np.inf)

EGO_A = 1.4
EGO_B = 1.4
EGO_MASS = 800
EGO_CR = 80000
EGO_TS = 0.1
N_PREDICTION = 35
N_CONTROL = 15
MAX_STEER = np.pi/3
DERIV_STEER = 0.07#*EGO_TS

class Controller2D(object):
    def __init__(self, waypoints, world):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 60.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        self._throttle_mean      = np.zeros(40,)

        # self.lateral_control = MPC(-80000,-100000,1000,1800,1.4,1.4,0.001,35,15,np.pi/3,0.0344)
        # self.lateral_control = NMPC(1.4,1.4,800,80000,0.1,35,15,np.pi/3,0.07)  #0.034
        self.lateral_control = NMPC(EGO_A, EGO_B, EGO_MASS, EGO_CR,
                                    EGO_TS, N_PREDICTION, N_CONTROL, MAX_STEER, DERIV_STEER)  
        self.velocity = None
        self.beta = None
        self.d_shi = None
        self.world = world
        
        self.vars.create_var("t_previous",0)
        self.vars.create_var("last_error",0)
        self.vars.create_var("tot_error",0)

        self._throttle_mean      = np.zeros(40,)
        self._brake_mean         = np.zeros(10,)

    def update_values(self, x, y, yaw, speed, timestamp, frame,velocity,beta,d_shi):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed+0.001
        self._current_timestamp = timestamp
        self._current_frame     = frame
        self.velocity = velocity
        self.beta = beta
        self.d_shi = d_shi
        if self._current_frame:
            self._start_control_loop = True
        
    def update_desired_speed(self):
        min_idx       = np.argmin(np.sum(np.square(self._waypoints[:,:2]-np.array([self._current_x,self._current_y])),axis=1))
        self._desired_speed =  self._waypoints[min_idx][2]

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints
        
    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    # def set_throttle(self, input_throttle):
    #     # Clamp the throttle command to valid bounds
    #     self._throttle_mean = self._throttle_mean[1:]
    #     self._throttle_mean = np.append(self._throttle_mean,np.array([input_throttle]))
    #     throttle = np.mean(self._throttle_mean)
    #     throttle           = np.fmax(np.fmin(throttle, 1.0), 0.0)
    #     self._set_throttle = throttle

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        self._throttle_mean = self._throttle_mean[1:]
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        throttle = np.mean(np.append(self._throttle_mean,np.array([throttle])))
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer_in_rad, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake
    

    # def set_brake(self, input_brake):
    #     # Clamp the brake command to valid bounds
    #     self._brake_mean = self._brake_mean[1:]
    #     self._brake_mean = np.append(self._brake_mean,np.array([input_brake]))
    #     brake = np.mean(self._brake_mean)
    #     brake           = np.fmax(np.fmin(brake, 1.0), 0.0)
    #     self._set_brake = brake


    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self.sample_wpts(self.lateral_control.Ts,self._waypoints,v,self.lateral_control.Np)

        # for i in range(waypoints.shape[0]):
        #     self.world.debug.draw_string(carla.Location(x=waypoints[i,0],y=waypoints[i,1],z=0.1), 'O', draw_shadow=False,
        #                                            color=carla.Color(r=0, g=0, b=255),life_time=0.03)


        throttle_output = 0
        steer_output    = 0
        brake_output    = 0
        error_t = 0

        # Skip the first frame to store previous values properly
        if self._start_control_loop:

            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            ##### IMPLEMENTATION OF LONGITUDINAL CONTROLLER ######
            ######################################################
            ######################################################
           
            Kp = 2#0.5
            Kd = 0.1#0.2
            Ki = 0.02#0

            delta_t = t - self.vars.t_previous

            error_t = v_desired - v
            # print(v_desired,v)
            d_error_t = (error_t - self.vars.last_error)
            self.vars.tot_error = (self.vars.tot_error*0.1) + error_t

            pid_val = Kp*error_t + Kd*d_error_t + Ki * self.vars.tot_error

            #constrain PID value between -1 and +1 .... change k_ according to the level of acceleration which you notice

            k_ = 0.5
            val = np.tanh(k_*pid_val)
            if(val>=0):
                throttle_output = val
                brake_output    = 0
            else:
                throttle_output = 0
                brake_output    = -2*val


            ######################################################
            ######################################################
            ####### IMPLEMENTATION OF LATERAL CONTROLLER  ########
            ######################################################
            ######################################################

            waypoints = waypoints[:self.lateral_control.Np,:2]
            Rot_mat = np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
            trans_mat = np.array([x,y])
            
            trans_waypoints = ((waypoints-trans_mat)@(Rot_mat.T))
            trans_waypoints[:,1] = -trans_waypoints[:,1]

            yaws = []
            for i in range(self.lateral_control.Np-1):
                yaw_wpt = np.arctan2(trans_waypoints[i+1][1]-trans_waypoints[i][1],trans_waypoints[i+1][0]-trans_waypoints[i][0])
                yaws.append(yaw_wpt)
            yaws.append(yaw_wpt)
            # yaws = (((np.array(yaws) + self._current_yaw)+np.pi)%(np.pi*2))-np.pi
            yaws = np.array(yaws)
            yaws = yaws.reshape((self.lateral_control.Np,1))
            trans_waypoints = np.append(trans_waypoints,yaws,axis=1)

            
            self.lateral_control.predict(0,0,0,self.velocity,self.beta)
            control = self.lateral_control.optimize(-self.d_shi,trans_waypoints)
            self.lateral_control.update_debug_costs(trans_waypoints)

            opt_states = self.lateral_control.get_Y(0,0,0,self.velocity,self.beta)
            opt_states[:,1] = -opt_states[:,1]
            temp = opt_states[:,:2]@Rot_mat + trans_mat
            # for kk in range(temp.shape[0]):
            #     self.world.debug.draw_string(carla.Location(x=temp[kk,0],y=temp[kk,1],z=0.1), 'O', draw_shadow=False,
            #                                        color=carla.Color(r=255, g=0, b=0), life_time=0.03)

            delta = control[0]
            steer_output    = -1*delta



            ######################################################
            ################# SET CONTROLS OUTPUT ################
            ######################################################
            self.set_throttle(throttle_output)  # in percent ( 0 to 1)
            self.set_steer(steer_output)        # in percent (-1 to )
            self.set_brake(brake_output)        # in percent ( 0 to 1)
            # return abs(trans_waypoints[0,0])
            
            
        ######################################################
        ######################################################
        ############# STORE OLD VALUES HERE ##################
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.t_previous = t
        self.vars.last_error = error_t
        self.vars.tot_error +=error_t
        if self._start_control_loop:
            return (trans_waypoints[0,1])   #returning y for current y

    def sample_wpts(self, Ts, wpts, v, Np):
        sample_dist = v*Ts
        sampled_wpt = np.empty((0,3))
        sampled_wpt = np.append(sampled_wpt,np.reshape(wpts[0],(1,3)),axis=0)
        last_wpt = 0
        for i in range(Np-1):
            dist = np.linalg.norm(sampled_wpt[-1,:2]-wpts[last_wpt+1,:2])
            last_count = last_wpt
            while dist < sample_dist:
                last_count += 1
                if last_count >= wpts.shape[0]-1:
                    ex_pt = wpts[-1] + (sample_dist*((wpts[-1]-wpts[-2])/np.linalg.norm(wpts[-1]-wpts[-2])))
                    wpts = np.append(wpts,np.reshape(ex_pt,(1,3)),axis=0)
                dist += np.linalg.norm(wpts[last_count,:2]-wpts[last_count+1,:2])
            last_wpt = last_count
            pt = wpts[last_wpt+1] - ((dist-sample_dist)*((wpts[last_wpt+1]-wpts[last_wpt])/np.linalg.norm(wpts[last_wpt+1]-wpts[last_wpt])))
            sampled_wpt = np.append(sampled_wpt,np.reshape(pt,(1,3)),axis=0)
        return sampled_wpt



