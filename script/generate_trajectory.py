# -*- coding: utf-8 -*-
#!/usr/bin/env python
from __future__ import division 
import math
import numpy as np
import matplotlib.pyplot as plt


# ROV Trajectory : quintic trajectory

def quintic_Trajectory(theta_init, theta_final, t, t_final) :
    thetad_init = 0
    thetad_final = 0
    thetadd_init = 0
    thetadd_final = 0
    a_0 = theta_init     #Polynomial parameter 
    a_1 = 0
    a_2 = 0
    #a_3 = (20 * theta_final - 20 * theta_init) / (2 * t_final**3) 
    # a_4 = (30 * theta_init - 30 * theta_final)  / (2 * t_final**4)
    # a_5 = (12 * theta_final - 12 * theta_init)  / (2 * t_final**5)
    a_3 = (20 * theta_final - 20 * theta_init - (8 * thetad_final + 12 * thetad_init) * t_final - (3 * thetadd_init - thetadd_final) * t_final**2 ) / (2 * t_final**3) #Polynomial parameter =  
    a_4 = (30 * theta_init - 30 * theta_final - (14 * thetad_final + 16 * thetad_init) * t_final + (3 * thetadd_final - 2 * thetadd_init) * t_final**2) / (2 * t_final**4)
    a_5 = (12 * theta_final - 12 * theta_init - (6 * thetad_final + 6 * thetad_init) * t_final - (thetadd_final - thetadd_init) * t_final**2) / (2 * t_final**5)
    
    if t <= t_final : 
        theta = a_0 + a_1 * t +  a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5         #Desired  trajectory(m)
        thetad = a_1  +  a_2 * t + 3 * a_3 * t**2 + 4 * a_4 * t**3 + 5* a_5 * t**4         #Desired velocity(m/s)
        thetadd = a_2 + 6 * a_3 * t + 12 * a_4 * t**2 + 20* a_5 * t**3                     #Desired acceleration(m/s^2)
           
    if t > t_final : 
        theta = theta_final
        thetad = 0
        thetadd = 0
        
    return theta, thetad, thetadd

# -------- simulation----------------- 

#Initialisation 
t_final = 6.5
Z_init = 0
Z_final = 1  #1.5708
iterr = 400

fs = 20
dt = 1/fs
Z_desired, Zd_desired, Zdd_desired = np.zeros((3, iterr))  

for i in range(iterr):
    
    Z_desired[i], Zd_desired[i], Zdd_desired[i] = quintic_Trajectory(Z_init , Z_final, i*dt, t_final) 

time = iterr * dt

#---------PLot--------------------------
fig, axs = plt.subplots(3, figsize=(10, 20), facecolor='w', edgecolor='k')
fig.subplots_adjust(hspace = .6, wspace=.001)

axs[0].plot(Z_desired)
axs[0].set_title("Trajectoire désirée")
axs[0].set(xlabel='time', ylabel='$x^* (m)$')

axs[1].plot(Zd_desired)
axs[1].set_title("Vitesse désirée")
axs[1].set(xlabel='time', ylabel='$u^* (m/s)$')

axs[2].plot(Zdd_desired)
axs[2].set_title("Accéleration désirée")
axs[2].set(xlabel='time', ylabel='$\dot{u}^*(m/s^2)$')
 

for ax in axs.flat:
    ax.label_outer()
    
plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9,
                    top=0.9, wspace=0.4, hspace=0.4) 
plt.show()
