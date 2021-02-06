#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

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
DERIV_STEER = 0.07

class Controller2D(object):
    def __init__(self, waypoints):
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
        #self._waypoints          = waypoints
        self._waypoints          = waypoints.tolist()
        self._conv_rad_to_steer  = 180.0 / 60.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

        # self.lateral_control = MPC(-80000,-100000,1000,1800,1.4,1.4,0.001,35,15,np.pi/3,0.0344)
        # self.lateral_control = NMPC(1.4,1.4,800,80000,0.1,35,15,np.pi/3,0.07)  #0.034
        self.lateral_control = NMPC(EGO_A, EGO_B, EGO_MASS, EGO_CR,
                                    EGO_TS, N_PREDICTION, N_CONTROL, MAX_STEER, DERIV_STEER)  
        self.velocity = None
        self.beta = None
        self.d_shi = None
        

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
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        # print(min_idx,len(self._waypoints)-1)
        
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        


        ######### VELOCITY PROFILES NEED TO BE GENERATED #################
        self._desired_speed =  desired_speed
        # if(desired_speed>15):

        #     self._desired_speed = desired_speed*1.2
        # else:
        #     self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        # self._waypoints = new_waypoints
        self._waypoints = new_waypoints.tolist()

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        # input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer_in_rad, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

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
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0
        error_t = 0
        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################

        self.vars.create_var("t_previous",0)
        self.vars.create_var("last_error",0)
        self.vars.create_var("tot_error",0)

        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        
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
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
           
            Kp = 1
            Kd = 2
            Ki = 0.001

            delta_t = t - self.vars.t_previous

            error_t = v_desired - v
            d_error_t = error_t - self.vars.last_error
            i_error_t = self.vars.tot_error + error_t

            pid_val = Kp*error_t + Kd*d_error_t + Ki*i_error_t

            #constrain PID value between -1 and +1 .... change k_ according to the level of acceleration which you notice
            # print(pid_val)
            k_ = 0.5
            val = np.tanh(k_*pid_val)
            # print(v_desired,v)
            if(val>=0):

                throttle_output = val
                brake_output    = 0
            
            else:

                throttle_output = 0
                brake_output    = -2*val   #this was 0.8


            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            # steer_output = 0

            # ke = 10
            # ks = 8


            # yaw_path = np.arctan2(waypoints[-1][1] - waypoints[0][1],waypoints[-1][0]- waypoints[0][0])
            # # print(yaw_path,yaw)
            # shi = yaw_path - yaw
            
            # if(shi > np.pi):
            #     shi-=2*np.pi
            # elif(shi< -np.pi):
            #     shi+= 2*np.pi
            # curr_ = np.array([x,y])
            
            # ###### taking closest point to the current pos and then taking the crosstrack error.

            # crosstrack_error = np.min(np.sum((curr_ - np.array(waypoints)[:, :2])**2, axis=1))

            # y1 = np.arctan2(y - waypoints[0][1],x- waypoints[0][0])

            # #checking if e(t) is +ve or -ve
            # temp = yaw_path - y1
            # if(temp > np.pi):
            #     temp-=2*np.pi
            # elif(temp< -np.pi):
            #     temp+= 2*np.pi

            # if temp > 0:
            #     crosstrack_error = crosstrack_error
            # else:
            #     crosstrack_error = -1*crosstrack_error
            
            # cross_error = np.arctan(ke*crosstrack_error/(ks+v))

            # delta = shi + cross_error
            # if delta > np.pi:
            #     delta -= 2 * np.pi
            # if delta < - np.pi:
            #     delta += 2 * np.pi

  
            # delta = max(-1.22,delta)
            # delta = min(1.22,delta)


            ###########################################################
            #                           NMPC                          #
            ###########################################################
            
            # waypoints = (np.array(self._waypoints[:])[:self.lateral_control.Np,:2])
            # waypoints[:,1] = waypoints[:,1]*-1 
            # # print(waypoints.shape,self.lateral_control.Np)
            # waypoints = (np.array(self._waypoints[:])[:self.lateral_control.Np,:2])
            # waypoints[:,1] = waypoints[:,1]*-1
            
            # yaws = []
            # for i in range(self.lateral_control.Np-1):
                
            #     # print(waypoints[i+1][1]-waypoints[i][1],waypoints[i+1][0]-waypoints[i][0])

            #     yaw = np.arctan2(waypoints[i+1][1]-waypoints[i][1],waypoints[i+1][0]-waypoints[i][0])

            #     if(yaw>np.pi):
            #         yaw-=2*np.pi
            #     elif(yaw<-np.pi):
            #         yaw+=2*np.pi
            #     yaws.append(yaw)
            
            # yaws.append(yaw)
            # yaws = np.array(yaws)
            # yaws = yaws.reshape((self.lateral_control.Np,1))

            # waypoints = np.append(waypoints,yaws,axis=1)
            
        
            # self.lateral_control.predict(self._current_x,-self._current_y,-self._current_yaw,self.velocity)
            # control = self.lateral_control.optimize(-self.d_shi,waypoints)

            # # print(control)
            # # print(Y[1,:],waypoints[0,:])
            # delta = self.lateral_control.PID(control[0],-self.d_shi)
            # steer_output    = -delta

            ###################################################################
            #                        MPC                                      #
            ###################################################################

            # waypoints = (np.array(self._waypoints[:])[:self.lateral_control.Np,:2]).T
            
            # # print(waypoints[:2,:3])
            
            # self.lateral_control.predict(self.velocity,0,-self._current_yaw,self.beta,-self.d_shi)

            # Rot_mat = np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
            # trans_mat = np.array([[x],[y]])
            # # print((Rot_mat@waypoints[:,0]).shape)
            # # print(-yaw)
            # trans_waypoints = (Rot_mat@(waypoints-trans_mat)).T


            
            # trans_waypoints = -(trans_waypoints[:,1]).reshape((self.lateral_control.Np,1))
            # # print(trans_waypoints[:3].reshape(1,3))
            # delta = self.lateral_control.new_optimize(trans_waypoints)
            # print(trans_waypoints[:10].reshape(1,10),self._current_timestamp)
            # print(waypoints[:,:10])
            
            # print(yaws[0],-self.d_shi,control[0],delta)

            # Change the steer output with the lateral controller. 
            # steer_output    = -delta[0]  #make negative

            ##################################################################
            #                       Lateral NMPC                              #
            ###################################################################

            waypoints = (np.array(self._waypoints[:])[:self.lateral_control.Np,:2])
      
            Rot_mat = np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
            # print(yaw)
            trans_mat = np.array([[x,y]])
            
            trans_waypoints = ((waypoints-trans_mat)@(Rot_mat.T))

            
            trans_waypoints[:,1] = -trans_waypoints[:,1]

            yaws = []

            for i in range(self.lateral_control.Np-1):

                yaw = np.arctan2(trans_waypoints[i+1][1]-trans_waypoints[i][1],trans_waypoints[i+1][0]-trans_waypoints[i][0])

                if(yaw>np.pi):
                    yaw-=2*np.pi
                elif(yaw<-np.pi):
                    yaw+=2*np.pi
                yaws.append(yaw)
            
            yaws.append(yaw)
            yaws = np.array(yaws)
            yaws = yaws.reshape((self.lateral_control.Np,1))

            trans_waypoints = np.append(trans_waypoints,yaws,axis=1)
            trans_waypoints = trans_waypoints[:,1:]

            
            self.lateral_control.predict(0,0,self.velocity,self.beta)
            control = self.lateral_control.optimize(-self.d_shi,trans_waypoints)

            delta = control[0]
            steer_output    = -delta

            # print(yaw,steer_output)
            # print(steer_output)
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """


            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

            # return abs(trans_waypoints[0,0])
            return trans_waypoints[0,0]
            
        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.last_error = error_t
        self.vars.tot_error +=error_t

        return 0
        
