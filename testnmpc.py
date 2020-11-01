from casadi import *
import numpy as np
import time 


class NMPC:

    def __init__(self,a,b,m,Cr,Ts,Np,Nc,max_steer,deriv_steer):

        self.a = a
        self.b = b
        self.Ts = Ts
        self.Np = Np
        self.Nc = Nc
        self.m = m
        self.Cr = Cr

        self.x_sym   = SX.sym("x")
        self.y_sym   = SX.sym("y")
        self.shi_sym = SX.sym("shi")
        self.v_sym   = SX.sym("v")
        
        self.last_error = 0
        self.tot_error = 0
        

        self.max_steer = max_steer
        self.deriv_steer = deriv_steer

        self.diag_Q = []
        self.diag_R = []
        self.diag_M = []

        self.K1 = self.m*self.a/((self.a+self.b)*self.Cr)
        self.K2 = self.b
        print(self.K1,self.K2)

        for i in range(self.Np):

            # self.diag_arr.append(np.e**-(0.05*i))
            # self.diag_R.append(np.e**-(0.001*i))
            self.diag_Q.append(1)
            if (i<self.Nc):
                self.diag_R.append(1)
                self.diag_M.append(1)

        self.diag_Q = SX(np.diag(self.diag_Q))
        self.diag_R = SX(np.diag(self.diag_R))
        self.diag_M = SX(np.diag(self.diag_M))

        self.pred_state_func = None
        self.U_k = SX.sym("d_shi_var", self.Nc,1)
        self.Y_k = None
        
        self.prev_U_k = 0


###############################################################################
#               Constrainst                                                   #
###############################################################################
        
        ###this needs to be updated each iteration

        ######### delta constrain ##################

        self.G_steer_pos = np.eye(self.Nc)
        self.G_steer_neg = -self.G_steer_pos
        
        self.G_steer_del_pos = -np.eye(self.Nc-1,self.Nc)+np.eye(self.Nc-1,self.Nc,1)
        self.G_steer_del_neg = -self.G_steer_del_pos
        
        self.G = None

        self.h_steer_del = np.full((2*(self.Nc-1),1),self.deriv_steer)
        self.h_steer = np.full((2*self.Nc,1),self.max_steer)

        self.h = None
 
    
    def predict(self, y, shi, v, beta):
        
        Y = SX(self.Np,2)

        for i in range(self.Np): 

            ###### disturbance real beta estimated beta
            if i==0:

                Y[0,0] = y
                Y[0,1] = shi
            
            elif(i<self.Nc):            

                Y[i,0] =   Y[i-1,0] + self.Ts*v*sin((self.K1*(v**2)*np.cos(beta)/(self.a+self.b) + self.K2*np.cos(beta)/(self.a+self.b))*tan(self.U_k[i-1,0]))
                Y[i,1] =   Y[i-1,1] + self.Ts*v*np.cos(beta)*tan(self.U_k[i-1,0])/(self.a + self.b)
            
            else:

                Y[i,0] =   Y[i-1,0] + self.Ts*v*sin((self.K1*(v**2)*np.cos(beta)/(self.a+self.b) + self.K2*np.cos(beta)/(self.a+self.b))*tan(self.U_k[self.Nc-1,0]))
                Y[i,1] =   Y[i-1,1] + self.Ts*v*np.cos(beta)*tan(self.U_k[self.Nc-1,0])/(self.a + self.b)

        self.Y_k = Y
        self.pred_state_func = Function('pred_state_func',[self.U_k],[self.Y_k])

    def optimize(self, d_shi, ref):

        prev_U_k = vertcat(self.prev_U_k,self.U_k[:-1])

        cost = 100000*(self.Y_k[:,0] - ref[:,0]).T@self.diag_Q@(self.Y_k[:,0] - ref[:,0]) \
                + 1000000*(self.Y_k[:,1] - ref[:,1]).T@self.diag_Q@(self.Y_k[:,1] - ref[:,1]) \
                + 100*(self.U_k.T@self.diag_R@self.U_k) \
                + 1*(self.U_k-prev_U_k).T@self.diag_M@(self.U_k-prev_U_k)
        
        var_pos = np.zeros((1,self.Nc))   ##to keep change consistent with previous timestep
        var_pos[0,0] = 1
        var_neg = np.zeros((1,self.Nc))
        var_neg[0,0] = -1

        self.G = SX(np.vstack((var_pos,var_neg,self.G_steer_pos,self.G_steer_neg,self.G_steer_del_pos,self.G_steer_del_neg)))

        nlp = {"x":self.U_k,"f":cost,"g":self.G@self.U_k}

        opts = {"ipopt.print_level": 0, "ipopt.print_timing_statistics":"no", "ipopt.print_info_string": "no", "print_time": False,"verbose" :False,"ipopt.sb":"yes"}
        solver = nlpsol('solver','ipopt',nlp, opts) 

        var_h_pos = self.deriv_steer +self.prev_U_k
        var_h_neg = self.deriv_steer-self.prev_U_k

        # print(self.var_h_pos,self.h_steer_pos[:4].T,self.var_h_neg,self.h_steer_neg[:4].T)

        self.h = DM(np.vstack((var_h_pos,var_h_neg,self.h_steer,self.h_steer_del)))

        r = solver(ubg = self.h)
        x_opt = r["x"]      

#####  Update these before ending the loop ###### 

        self.prev_U_k = x_opt[0]

        return x_opt
    
    def get_pred_states(self, y, shi, v, pred_control):
        return self.pred_state_func(pred_control)

    def PID(self,expected_yaw,real_d_shi):
        
        
        error = expected_yaw - real_d_shi

        delta = self.Kp*error + self.Kd*(error-self.last_error)/self.Ts + self.Ki*(error*self.Ts+self.tot_error)
        # print(error,(error-self.last_error)/self.Ts , error*self.Ts+self.tot_error)
        self.last_error = error
        self.tot_error += self.Ts*error

        # print(expected_yaw,real_d_shi,delta,"a")
        return delta

