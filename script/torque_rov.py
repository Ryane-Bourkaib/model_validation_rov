#!/usr/bin/env python
from __future__ import division 
import math
import numpy as np
import matplotlib.pyplot as plt
from Parameters import parameters
import stopp
from generate_trajectory import quintic_Trajectory

para = parameters()
# ---------- Torque function ------------------------------------
# This function used to calculate the  generelized forces by using the dynamic model 
def torque_rov(Acc_B, PosE, Speed_B):
    #----------- HYDROSTATIC : RESTORING FORCE VECTOR -----------
    phi = PosE[3]             #Oriontation Roll
    theta = PosE[4]           #Oriontation Pitch
    #Restoring force vector in Body 
    g = [(para.W - para.B) * np.sin(theta),
         -(para.W - para.B) * np.cos(theta) * np.sin(phi),
         -(para.W - para.B) * np.cos(theta) * np.cos(phi),
         para.zg * para.W * np.cos(theta) * np.sin(phi),
         para.zg * para.W * np.sin(theta), 0]
    
    # -----------CORIOLIS FORCE C_b -----------
    u = Speed_B[0]          #Velocity surge (m/s^2)
    v = Speed_B[1]          #Velocity sway  (m/s^2)     
    w = Speed_B[2]          #Velocity heave (m/s^2)
    p = Speed_B[3]          #Velocity roll  (rad/s^2)
    q = Speed_B[4]          #Velocity pitch (rad/s^2)
    r = Speed_B[5]          #Velocity yaw   (rad/s^2)
   
    C_b = np.array([[0, 0, 0, 0, para.m * w, 0],
           [0, 0, 0, -para.m * w, 0, 0],
           [0, 0, 0, para.m * v, -para.m * u, 0],
           [0, para.m * w, -para.m * v, 0, para.Iz * r, -para.Iy * q],
           [-para.m * w, 0, -para.m * u, -para.Iz * r, 0, para.Ix * p],
           [para.m * v, -para.m * u, 0, para.Iy * q, -para.Ix * p, 0]])
    
    #----------- NON LINEAR HYDRODYNAMIC CORIOLIS AND CENTRIPAL MATRIX C_A -----------
   
    C_A = np.array([[0, 0, 0, 0, para.Z_wd * w, 0],
           [0, 0, 0, -para.Z_wd * w, 0, -para.X_ud * u],
           [0, 0, 0, -para.Y_vd * v, para.X_ud * u, 0],
           [0, -para.Z_wd * w, para.Y_vd * v, 0, -para.N_rd * r, para.M_qd * q],
           [para.Z_wd * w, 0, -para.X_ud * u, para.N_rd * r, 0, -para.K_pd * p],
           [-para.Y_vd * v, para.X_ud * u, 0, -para.M_qd * q, para.K_pd * p, 0]])
        
    # -----------RELATIVE VELOCITY VECTOR -----------
    #V_c = [u, v, w, 0, 0, 0]        #Current disturbance 
    #V_w = Speed_B - np.array(V_c)   #Relative velocity vector 
    V_w = Speed_B
    # -----------FORCES MATRIX -----------
   
    ##Curiolis forces
    F_c = C_b.dot(Speed_B)
    F_cAdded = C_A.dot(V_w) 
    
    ##Damping force 
    F_D = para.D.dot(V_w)
    #generelized forces and torques 
    torque =  para.M.dot(Acc_B) + F_c + F_cAdded + F_D + g     
    return torque

# ----------- Calculate the pwm from a force value -------------------------------
def PWM_Cmd(torque):
    if (torque > 0 and torque> abs(10**(-2))) :
        m = 86.93393326839376   # Slope of the positive force linear function
        b = 1536
    elif (torque < 0 and torque < abs(10**(-2))):
        m = 110.918185437553874 # Slope of the negtaive force linear function
        b = 1464
    else : 
        m = 0
        b = 1500
    PWM = int(m * torque/4) + b 
    if PWM >1900:
        PWM = 1900
    if PWM < 1100:
        PWM = 1100
    return PWM


# ---------------------------SIMULATION --------------------------------------------
# testing the forwerd motion
# Initialisation 
t_final = 10  # desired final time 
u_init = 0   # initialposition 
u_final = 1  # desired final position 
iterr = 600  # itteration
fs = 20      # frequency 
dt = 1/fs    # step time 

x, u, ud = np.zeros((iterr)),np.zeros((iterr)), np.zeros((iterr)) 
pwm_surge = np.zeros((iterr))
forces, Acc = np.zeros((iterr, 6)), np.zeros((iterr, 6))

for i in range(iterr):  
    x[i], u[i], ud[i] = quintic_Trajectory(u_init , u_final, i*dt, t_final) 
    Acc_B = np.array([ud[i], 0, 0, 0, 0, 0])
    Pos = np.array([x[i], 0, 0, 0, 0, 0])
    Speed_B = np.array([u[i], 0, 0, 0, 0, 0])
    
    tau = torque_rov(Acc_B, Pos, Speed_B)
    forces[i, :] = tau
    Acc[i, :] = Speed_B
    pwm_surge[i] = PWM_Cmd(tau[0])



#---------------------Position PLot -------------------------------------
# fig1, axs = plt.subplots(6, figsize=(10, 20), facecolor='w', edgecolor='k')
# fig1.subplots_adjust(hspace = .6, wspace=.001)

# for i in range(6):
#     axs[i].plot(forces[:,i])
#     axs[i].set_title(str(i))
# for ax in axs.flat:
#     ax.set(xlabel='time(s)', ylabel='force(N)')

# for ax in axs.flat:
#     ax.label_outer()
    
# plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9,
#                     top=0.9, wspace=0.4, hspace=0.4) 

# # plt.show()
# # plt.plot(pwm_surge)

# plt.show()
