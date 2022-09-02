# -*- coding: utf-8 -*-
#!/usr/bin/env python

#Libraries
from __future__ import division 
import math
import os
import numpy as np
from scipy import linalg

class parameters:
    def __init__(self):

        #Initialiation :
        self.m = 11.5							         #Mass of blueRov (Kg)
        self.g = 9.81								     #Earth gravity(m*s^(-2))
        self.rho = 1000                                  #Water density
        self.W = self.m * self.g                         #Weight (N)
        self.L = 0.475                                   #Lenghth of blueRov( m)
        self.Net_Buoyancy = 0.2                          #Ne bouyancy (kg f)                              
        self.B = self.W + self.g * self.Net_Buoyancy     #Buoancy Force (N)
        
		
        #Center of gravity in Body Frame (CG in CO)
        self.xg = 0
        self.yg = 0
        self.zg = 0.02 
        #self.rg = [self.xg, self.yg, self.zg]         #Vecteur des cood du centre gravitationnel (we suppose CO=CG) (m)
        
        #Buoyancy position in body frame (CB in CO)
        self.xb = 0
        self.yb = 0
        self.zb = 0
        #self.rb = [self.xb, self.yb, self.zb]         #Vecteur de position de force d'archimède
        
        
        ##############################INERTIA MATRIX IN CO (M_b)#####################################
        #Inertia coef for a vehicul in CG (using )
        self.Ix = 0.16                #Ineria of BlueRov along x (km m^2)
        self.Iy = 0.16               #Ineria of BlueRov along y (km m^2)
        self.Iz = 0.072    #Ineria of BlueRov along z (km m^2)
        self.Ib = [[self.Ix, 0, 0],   #Matrix of inertia in center of vehicul in CG=CO (0.16)
                   [0, self.Iy, 0],
                   [0, 0,self.Iz]]
        
        self.M_b = np.array([[self.m, 0, 0, 0, self.m * self.zg, 0],
                    [0, self.m, 0, -self.m * self.zg, 0, 0],
                    [0, 0, self.m, 0, 0,0],
                    [0, -self.m * self.zg, 0, self.Ix, 0, 0],
                    [self.m * self.zg, 0, 0, 0, self.Iy, 0],
                    [0, 0, 0, 0, 0, self.Iz]])
     
        
        

        ########################## ADDED MASS M_A #################################################"
        self.X_ud = -2.25                 #Added mass in surge(kg) 5.5 (with the other one is 5)
        self.Y_vd = -10.7               #Added mass in sway (kg) 12.7 
        self.Z_wd = -14.57                #Added mass in heave(kg)
        self.K_pd = -0.12                 #Added mass in roll (kg m^2/rad)
        self.M_qd = -0.12                #Added mass in pitch(kg m^2/rad)
        self.N_rd = -0.12                #Added mass in yaw  (kg m^2/rad) 0.12
        self.M_A = [[-self.X_ud, 0, 0, 0, 0, 0],
                    [0, -self.Y_vd, 0, 0, 0, 0],
                    [0, 0, -self.Z_wd, 0, 0, 0],
                    [0, 0, 0, -self.K_pd, 0, 0],
                    [0, 0, 0, 0, -self.M_qd, 0],
                    [0, 0, 0, 0, 0, -self.N_rd]]
        
        ############# GENERELIZED MASS MATRIX IN CO (M) ##################################### 
        self.M = np.array(self.M_b) + np.array(self.M_A)
        
        ##################### LINEAR DAMPING MATRIX D_L #############################################
        #These values are calculated experimentally ( cite : articl file:///C:/Users/ryane/OneDrive/Desktop/STAGE/documentations/ThesisWu2018.pdf )
        self.kl_u = -8.359       #damp parameter "Surge" (Ns/m) 8 works 
        self.kl_v = -14.22       #damp parameter "Sway" (Ns/m) 6.22
        self.kl_w = -5.18        #damp parameter "Heave" (Ns/m)
        self.kl_p = -0.07        #damp parameter "Roll" (Ns/rad)
        self.kl_q = -0.07        #damp parameter "Pitch" (Ns/rad)
        self.kl_r = -0.5     #damp parameter "Yaw" (Ns/rad) 0.15
        self.D_L = [[-self.kl_u, 0, 0, 0, 0, 0],
                    [0, -self.kl_v, 0, 0, 0, 0],
                    [0, 0, -self.kl_w, 0, 0, 0],
                    [0, 0, 0, -self.kl_p, 0, 0],
                    [0, 0, 0, 0, -self.kl_q, 0],
                    [0, 0, 0, 0, 0, -self.kl_r]]
        
        
         
        ##################### QUADRATIC DAMPING MATRIX D_Q#############################################
        #These values are calculated experimentally ( cite : articl file:///C:/Users/ryane/OneDrive/Desktop/STAGE/documentations/ThesisWu2018.pdf )
        self.kq_u = -18.18        #damp parameter "Surge"(Ns^2/m)
        self.kq_v = -21.66        #damp parameter "Sway" (Ns^2/m)
        self.kq_w = -36.99        #damp parameter "Heave"(Ns^2/m)
        self.kq_p = -1.55         #damp parameter "Roll" (Ns^2/rad)
        self.kq_q = -1.55         #damp parameter "Pitch"(Ns^2/rad)
        self.kq_r = -0.05         #damp parameter "Yaw"  (Ns^2/rad)
        self.D_Q = [[-self.kq_u, 0, 0, 0, 0, 0],
                    [0, -self.kq_v, 0, 0, 0, 0],
                    [0, 0, -self.kq_w, 0, 0, 0],
                    [0, 0, 0, -self.kq_p, 0, 0],
                    [0, 0, 0, 0, -self.kq_q, 0],
                    [0, 0, 0, 0, 0, -self.kq_r]]
        
        ##################### DAMPING MATRIX D #############################################
        #We negleged the quadratic damping because we work in slow speed, 
        # self.D = np.array(self.D_L) + np.array(self.D_Q)  
        self.D = np.array(self.D_L)  
        self.D_x = self.D[0,0]
        self.D_y = self.D[1,1]
        self.D_z = self.D[2,2]
        self.D_phi = self.D[3,3]
        self.D_theta = self.D[4,4]
        self.D_psi = self.D[5,5]
        ######################### THRUSTER MODEL ################################"
        #These parameters are calculated experimentally (cite : file:///C:/Users/ryane/OneDrive/Desktop/STAGE/documentations/ThesisWu2018.pdf)
        
        #Moment arms of 8 thrusters in body frame:
        self.l_x1 = 0.156      #moment arm thrust 1 along x (m)
        self.l_y1 = 0.111      #moment arm thrust 1 along y (m)
        self.l_z1 = 0.085      #moment arm thrust 1 along z (m)
        self.l_x2 = 0.156      #moment arm thrust 2 along x (m)
        self.l_y2 = -0.111     #moment arm thrust 2 along y (m)
        self.l_z2 = 0.085      #moment arm thrust 2 along z (m)
        self.l_x3 = -0.156     #moment arm thrust 3 along x (m)
        self.l_y3 = 0.111      #moment arm thrust 3 along y (m)
        self.l_z3 = 0.085      #moment arm thrust 3 along z (m)
        self.l_x4 = -0.156     #moment arm thrust 4 along x (m)
        self.l_y4 = -0.111     #moment arm thrust 4 along y (m)
        self.l_z4 = 0.085      #moment arm thrust 4 along z (m)
        self.l_x5 = 0.120      #moment arm thrust 5 along x (m)
        self.l_y5 = 0.218      #moment arm thrust 5 along y (m)
        self.l_z5 = 0          #moment arm thrust 5 along z (m)
        self.l_x6 = 0.120      #moment arm thrust 6 along x (m)
        self.l_y6 = -0.218      #moment arm thrust 6 along y (m)
        self.l_z6 = 0          #moment arm thrust 6 along z (m)
        self.l_x7 = -0.120     #moment arm thrust 7 along x (m)
        self.l_y7 = 0.218     #moment arm thrust 7 along y (m)
        self.l_z7 = 0          #moment arm thrust 7 along z (m)
        self.l_x8 = -0.120     #moment arm thrust 8 along x (m)
        self.l_y8 =-0.218      #moment arm thrust 8 along y (m)
        self.l_z8 = 0          #moment arm thrust 8 along z (m)
        
        # #Thrust matrix configurations 
        self.t1 = [0.707, -0.707, 0, 0.06, 0.06, -0.1888]
        self.t2 = [0.707, 0.707, 0, -0.06, 0.06, 0.1888]
        self.t3 = [-0.707, -0.707, 0, 0.06, -0.06, 0.1888]
        self.t4 = [-0.707, 0.707, 0, -0.06, -0.06, -0.1888]
        self.t5 = [0, 0, 1, -0.218, 0.120, 0]     # change the sign for motor 5
        self.t6 = [0, 0, 1, -0.218, -0.120, 0]  
        self.t7 = [0, 0, 1, 0.218, 0.120, 0] 
        self.t8 = [0, 0, 1, 0.218, -0.120, 0]  #chage sign of motor 8

         #Thrust matrix configurations 
        # self.t1 = [-0.707, +0.707, 0, -0.06, -0.06, 0.1888]
        # self.t2 = [-0.707, -0.707, 0, 0.06, -0.06, -0.1888]
        # self.t3 = [0.707, 0.707, 0, -0.06, 0.06, -0.1888]
        # self.t4 = [0.707, -0.707, 0, 0.06, 0.06, 0.1888]
        # self.t5 = [0, 0, 1, 0.218, -0.120, 0]
        # self.t6 = [0, 0, -1, 0.218, 0.120, 0]
        # self.t7 = [0, 0, -1, -0.218, -0.120, 0]
        # self.t8 = [0, 0, 1, -0.218, 0.120, 0]
       
        self.T = np.transpose([self.t1, self.t2, self.t3, self.t4, 
                               self.t5, self.t6, self.t7, self.t8])
        self.T_inv = np.linalg.pinv(self.T)
      
        #Thruster gains 
        self.K_t1 = 40  
        self.K_t2 = 40
        self.K_t3 = 40
        self.K_t4 = 40
        self.K_t5 = 40
        self.K_t6 = 40
        self.K_t7 = 40
        self.K_t8 = 40
        self.K = [[self.K_t1, 0, 0, 0, 0, 0, 0, 0],
                  [0, self.K_t2, 0, 0, 0, 0, 0, 0],
                  [0, 0, self.K_t3, 0, 0, 0, 0, 0],
                  [0, 0, 0, self.K_t4, 0, 0, 0, 0],
                  [0, 0, 0, 0, self.K_t5, 0, 0, 0],
                  [0, 0, 0, 0, 0, self.K_t6, 0, 0],
                  [0, 0, 0, 0, 0, 0, self.K_t7, 0],
                  [0, 0, 0, 0, 0, 0, 0, self.K_t8]]
        self.K_inv = np.linalg.inv(self.K)
        # self.T_inv =np.array([[ 3.53606789e-01, -3.53606789e-01, 0, 0, 0, -1.32415254e+00],
        #         [ 3.53606789e-01, 3.53606789e-01, 0, 0, 0,  1.32415254e+00],
        #         [-3.53606789e-01,-3.53606789e-01, 0,  0, 0,  1.32415254e+00],
        #         [-3.53606789e-01, 3.53606789e-01, 0, 0, 0, -1.32415254e+00],
        #         [0, 0,  2.50000000e-01, -1.14678899e+00,  2.08333333e+00,  0],
        #         [0, 0,  2.50000000e-01, -1.14678899e+00, -2.08333333e+00,  0],
        #         [0, 0,  2.50000000e-01,  1.14678899e+00, 2.08333333e+00, 0],
        #         [0, 0,  2.50000000e-01,  1.14678899e+00, -2.08333333e+00, 0]])

        self.T_inv =np.array([[-3.93606789e-01, 3.93606789e-01, 0, 0, 0, 0.7], #change the value of yaw gain = 1.32415254e+00
                [ -3.93606789e-01, -3.93606789e-01, 0, 0, 0,  -0.7],
                [3.93606789e-01,3.93606789e-01, 0,  0, 0,  -0.7],     #incearse the swaw gain ( 0.353 to 0.393)
                [3.93606789e-01, -3.93606789e-01, 0, 0, 0, 0.7],      #incearse the swa gain ( 0.353 to 0.398)
                [0, 0,  -2.50000000e-01, -0.5,  -0.508333333e+00,  0],  # decrease the values of the pitch gain (2.08 to 1.0833)
                [0, 0,  -2.50000000e-01, -0.5, 0.508333333e+00,  0], # decrease the values of the roll gain (1.4 to 0.)
                [0, 0,  -2.50000000e-01,  0.5, -0.508333333e+00, 0],
                [0, 0,  -2.50000000e-01,  0.5, 0.508333333e+00, 0]])
        
        self.conf_matrix = np.array([[-6.35151646e-01, -6.35151646e-01, 6.35151646e-01, 6.35151646e-01, 0, 0, 0, 0], 
                        [6.35151646e-01, -6.35151646e-01, 6.35151646e-01, -6.35151646e-01, 0, 0, 0, 0],
                        [0, 0, 0, 0, -1.00000000e+00, -1.00000000e+00, -1.00000000e+00, -1.00000000e+00],
                        [0, 0, 0, 0, -5.00000000e-01, -5.00000000e-01,  5.00000000e-01, 5.00000000e-01],
                        [0, 0, 0, 0, -4.91803279e-01, 4.91803279e-01, -4.91803279e-01, 4.91803279e-01],
                        [3.57142857e-01, -3.57142857e-01, -3.57142857e-01, 3.57142857e-01, 0, 0, 0, 0]])
       # self.Kinv_dot_Tinv = self.K_inv @ self.T_inv
       
pass

para = parameters()
# print(np.shape(np.linalg.pinv(para.T)))
# print(np.linalg.pinv(para.T_inv))

