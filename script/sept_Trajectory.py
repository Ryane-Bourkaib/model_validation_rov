# -*- coding: utf-8 -*-
#!/usr/bin/env python
from __future__ import division 
import math
import numpy as np
import matplotlib.pyplot as plt

#------------Symetric acceleration 
#ROV Trajectory : quintic trajectory
def sept_Trajectory(theta_init, theta_final, t, final_time ):
    
    if t<=final_time:
        theta = theta_final * t**4/ final_time**4 * (35 - 84 * t/final_time + 70 * t**2/ final_time**2 - 20 * t**3/final_time**3) 
        thetad = 140 * theta_final / final_time**2 * t**2/ final_time**3 * (1 - 3 * t/final_time + 3 * t**2/ final_time**2 - t**3/final_time**3) 
        thetadd = 140 * theta_final / final_time**2 * t**2/ final_time**2 * (3 - 12 * t/final_time + 15 * t**2/ final_time**2 - 6 * t**3/final_time**3) 
        thetaddd =  840 * theta_final/final_time**3 * t / final_time * (1 - 6 * t/final_time + 10 * t**2/ final_time**2 - 5 * t**3/final_time**3) 
    
    if t>= final_time:
        theta = theta_final
        thetad = 0
        thetadd = 0
        thetaddd = 0
    return theta, thetad, thetadd, thetaddd

# #------------Diffrent acceleration
# #ROV Trajectory : quintic trajectory
# def sept_Trajectory(theta_init, theta_final, t, final_time ):
#     t_1 = 2.5
#     t_finall = 5
    
#     theta = theta_final * t**4/ t_finall**4 * (35 - 84 * t/t_finall + 70 * t**2/ t_finall**2 - 20 * t**3/t_finall**3) 
#     thetad = 140 * theta_final / t_finall**2 * t**2/ t_finall**3 * (1 - 3 * t/t_finall + 3 * t**2/ t_finall**2 - t**3/t_finall**3) 
   
#     if t<=t_1:
#         thetadd = 140 * theta_final / final_time**2 * t**2/ final_time**2 * (3 - 12 * t/final_time + 15 * t**2/ final_time**2 - 6 * t**3/final_time**3) 
#         thetaddd =  840 * theta_final/final_time**3 * t / final_time * (1 - 6 * t/final_time + 10 * t**2/ final_time**2 - 5 * t**3/final_time**3) 
    
        
#     if t< t_finall and t > t_1:
#         #theta = theta_final * t**4/ t_finall**4 * (35 - 84 * t/final_time + 70 * t**2/ final_time**2 - 20 * t**3/final_time**3)  #70
#         thetad = 140 * theta_final / final_time**2 * t**2/ final_time**3 * (1 - 3 * t/final_time + 3 * t**2/ final_time**2 - t**3/final_time**3) 
#         thetadd = 70 * theta_final / t_finall**2 * t**2/ t_finall**2 * (3 - 12 * t/t_finall + 15 * t**2/ t_finall**2 - 6 * t**3/t_finall**3) 
#         thetaddd = 840 * theta_final/t_finall**3 * t / t_finall * (1 - 6 * t/t_finall + 10 * t**2/ t_finall**2 - 5 * t**3/t_finall**3) 

#     if t>= t_finall:
#         theta = theta_final
#         thetad = 0
#         thetadd = 0
#         thetaddd = 0
#     return theta, thetad, thetadd, thetaddd



# #Ploting : 
# #Initialisation 
# t_final = 5
# Z_init = 0
# Z_final = 1.5708  #1.5708
# iterr = 600
# fs = 0
# dt = 1/1
# Z_desired, Zd_desired, Zdd_desired, Zddd_desired  = np.zeros((4, iterr))  

# for i in range(iterr):   
#     Z_desired[i], Zd_desired[i], Zdd_desired[i], Zddd_desired[i] = sept_Trajectory(Z_init , Z_final, i*0.05, t_final) 


# # # ###################PLot#####################################
# # fig, axs = plt.subplots(4, figsize=(10, 20), facecolor='w', edgecolor='k')
# # fig.subplots_adjust(hspace = .6, wspace=.001)

# # axs[0].plot(Z_desired)
# # axs[0].set_title("Position désirée")
# # axs[0].set_title("Position désirée")
# # axs[0].set(xlabel='time', ylabel='pos (m)')

# # axs[1].plot(Zd_desired)
# # axs[1].set_title("Vitesse désirée")
# # axs[1].set(xlabel='time', ylabel='vel(m/s)')

# # axs[2].plot(Zdd_desired)
# # axs[2].set_title("Accéleration désirée")
# # axs[2].set(xlabel='time', ylabel='acc(m/s^2)')


# # axs[3].plot(Zddd_desired)
# # axs[3].set_title("jerk")
# # axs[3].set(xlabel='time', ylabel='acc(m/s^2)')
 

# # for ax in axs.flat:
# #     ax.label_outer()
    
# # plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9,
# #                     top=0.9, wspace=0.4, hspace=0.4) 
# # plt.show()
