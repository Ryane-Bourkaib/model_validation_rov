#!/usr/bin/env python
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

#Sliding mode controller 
Dw = 5.2           #Linear dampping coef
m = 11.5            #ROV mass (kg)
def SM_Controller(desired_Trajectory, real_Trajectory, desired_speed, real_Speed, rho, K, phi):
    global fs

    #Errors :
    Z_tilde = desired_Trajectory - real_Trajectory  #Depth error
    W_tilde = desired_speed - real_Speed            #Speed error
    # print(desired_Trajectory)
    # print(real_Trajectory)
    #SM surface : 
    S = rho[0] * Z_tilde + W_tilde *rho[1] + 0.01


    #Command with chatering 
    #f_z = -(m * rho[0] - Dw) * W_tilde - K * m * np.arctan(S)

    #Command without chatering 
    if (np.abs(S) < phi) : 
        f_z = -(m * rho[0] - Dw) * W_tilde - K * m * S / phi
    else :
        f_z = -(m * rho[0] - Dw) * W_tilde - K * m *np.sign(S / phi) 

    return f_z


########################TEST###########################################
# def model(Y, t,dy):
#    solution = Y                                 #Solution of the differeyial equation
#    dYdt =  dy                                   #Differential equation 
#    return dYdt

# #Initialisation :
# W0 = 0              #Initial heave velocity 
# Z0 = 0              #Initial depth 
# fs = 20             #Frequence 
# dt = 1/fs           #Step time 
# time = 300
# desired_Trajectory, desired_speed = np.zeros((2, time))  
# real_Trajectory, real_speed = np.zeros((2, time))  
# Zd_init = 0         #initial desired trajectory 
# Zd_final = 0.5      #desired trajectory (0.5 m)
# rho =np.array([19.99665,1])
# K = 10.99665
# phi = 0.989665

# for i in range(time): 
#     #Desired speed and trajectory 
#     desired_Trajectory[i], desired_speed[i] = Rov_Trajectory (Zd_init, Zd_final, i*0.05, 10)
    
#     Dt = [i, i+dt]      #interval of integration 
    
#     #Sliding mode control :
#     f_z = -SM_Controller(np.array(desired_Trajectory[i]), Z0, np.array(desired_speed[i]), W0, rho ,K , phi)  
#     print(f_z)
#     #Rov model along the heave :
#     W_dot = 1 / m * (f_z - Dw * W0)                 #Acceleration along the heave 
#     W = odeint(model, W0, Dt, args=( W_dot,))       #Speed along the heave
    
#     W0 = W[-1]
#     Z = odeint(model, Z0, Dt, args=( W0,))
#     Z0 = Z[-1]

#     real_Trajectory[i] = Z0
#     real_speed[i] = W0


# #Ploting :
# plt.plot(real_Trajectory, label = "Depth")
# plt.plot(desired_Trajectory,label = "Desired depth")
# plt.xlabel("time(s)")
# plt.ylabel("real and desired Depth (m)")
# plt.legend()
# plt.show()
