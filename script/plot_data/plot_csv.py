import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import json 

namespace = 'br4'
filename_distance = "_slash_" + namespace + "_slash_distance_sonar.csv"
filename_angle = "_slash_" + namespace + "_slash_angle_degree.csv"
filename_force = "_slash_" + namespace + "_slash_surge_force.csv"
filename_pwm = "_slash_" + namespace + "_slash_mavros_slash_rc_slash_override.csv"

file_path = os.path.abspath(os.getcwd())  
os.listdir()
# folder_name = 'good_yaw_Control_Kp_psi0.0149_Ki_psi=0.0_Kd_psi=0_2022-05-30-01-54-27/'
folder_name = 'ModelValidationDamping_good_Kp_psi0.0149_Ki_psi=0.0_Kd_psi=0_2022-05-30-05-56-28/'

#--------Pinger data----------#
data_distance = pd.read_csv(file_path + '/bags/data_measurements/' + folder_name + filename_distance)
data_timestamp_distance = data_distance.loc[:,'rosbagTimestamp']
data_offset = data_distance.loc[:,'data_offset']

distance = np.zeros((len(data_distance)))
for i, data_offset_element in enumerate(data_offset):
   value = json.loads(data_offset_element)[0]
   distance[i] = float(value)      #float(value.split(",")[0].replace("[", ""))

#--------IMU data----------#
data_angle = pd.read_csv(file_path + '/bags/data_measurements/' + folder_name + filename_angle)
data_yaw_angle =  data_angle.loc[:,'z']
data_timestamp_angle = data_angle.loc[:,'rosbagTimestamp']

#--------force data----------#
data_force = pd.read_csv(file_path + '/bags/data_measurements/' + folder_name + filename_force)
data_timestamp_force = data_force.loc[:,'rosbagTimestamp']
data_force = data_force.loc[:,'data']

#--------PWM data----------#
data_pwm = pd.read_csv(file_path + '/bags/data_measurements/' + folder_name + filename_pwm)
data_timestamp_pwm = data_pwm.loc[:,'rosbagTimestamp']
data_pwm = data_pwm.loc[:,'channels']

pwm = np.zeros((len(data_pwm)))
for i, data_pwm_element in enumerate(data_pwm):
   value = json.loads(data_pwm_element)[4]
   pwm[i] = float(value)      #float(value.split(",")[0].replace("[", ""))
  


#----------Ploting-------------
fig, axs = plt.subplots(4, figsize=(10, 20), facecolor='w', edgecolor='k')
fig.subplots_adjust(hspace = .6, wspace=.001)

axs[0].plot(data_timestamp_distance, distance)
axs[0].set_title("Distance mesurée par le pinger ")
axs[0].set(xlabel='time', ylabel='distance (mm)')

axs[1].plot(data_timestamp_angle, data_yaw_angle)
axs[1].set_title("Asservissment de l'angle du lacet")
axs[1].set(xlabel='time', ylabel='yaw (degree)')

axs[2].plot(data_timestamp_force, data_force)
axs[2].set_title("Force generalisée au long du x calculée par le modèle dynamique ")
axs[2].set(xlabel='time', ylabel='force(N)')
 
axs[3].plot(data_timestamp_pwm, pwm)
axs[3].set_title("PWM envoyer aux moteurs correspond au mouvement d'avancment")
axs[3].set(xlabel='time', ylabel='pwm(us)')

# for ax in axs.flat:
#     ax.label_outer()
    
# plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9,
#                     top=0.9, wspace=0.4, hspace=0.4) 
plt.show()


# plt.plot(data_timestamp_distance, distance)
# plt.ylabel("distance (mm)")
# plt.xlabel("temps (s)")
# plt.title("Distance mesurée par le pinger ")
# plt.show()

# plt.plot(data_timestamp_angle, data_yaw_angle)
# plt.ylabel(" yaw (degré)")
# plt.xlabel("temps (s)")
# plt.title("L'angle de lacet ")
# plt.show()

# plt.plot(data_timestamp_force, data_force)
# plt.ylabel("force (N)")
# plt.xlabel("temps (s)")
# plt.title("Force generalisée au long du x calculée par le modèle dynamique ")
# plt.show()


plt.plot(data_timestamp_pwm, pwm)
plt.ylabel("PWM (us)")
plt.xlabel("temps (s)")
plt.title("PWM envoyer aux moteurs correspond au mouvement d'avancment")
plt.show()