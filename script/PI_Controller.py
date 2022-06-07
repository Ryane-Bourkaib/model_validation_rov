#!/usr/bin/env python
import os
import math
import numpy as np
import matplotlib.pyplot as plt



def PID_Controller(x_desired, x_real, K_P, K_I, K_D, e_0, I0, step,flotability, r = None):
    
    e =  x_desired - x_real                 # error between the real and desired value
    P = K_P * e                             # proportional controller 
    if r == None : 
        D = K_D * ((e - e_0)/ step)         # derivative controller 
    else : 
        D = K_D * r                         # derivative controller using a measured velocity 

    I = I0 +  e * step                       #Integral controller

    PID_Controller = P + K_I * I + D + flotability                     #Output of the PID controller 
    e_0 = e                              #Update the error value  
    I0 = I                               #Update the initial value of integral controller 

    return PID_Controller 



def yaw_Controller(yaw_d, yaw_m, r, Kp_psi, Ki_psi, Kd_psi, I0_psi, step):
    e_yaw = yaw_d - yaw_m
    if e_yaw <= -180:  
        e_yaw = e_yaw + 360
    if e_yaw > 180 :
        e_yaw = e_yaw - 360
    I = I0_psi +  e_yaw * step  
    yaw_torque = -(Kp_psi * e_yaw + Ki_psi * I) - Kd_psi * r 
    I0_psi = I
    return yaw_torque