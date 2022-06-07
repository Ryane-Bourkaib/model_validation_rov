from ctypes import sizeof
import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt
import os
import sys 

# namespace = 'br5'
# filename = "_slash_" + namespace + "_slash_mavros_slash_imu_slash_water_pressure.csv"
# file_path = os.path.abspath(os.getcwd())  
# print(file_path)
# os.listdir()
# data = pd.read_csv(file_path + '/autonomous_rov/script/data_measurements/' + filename)
# print(data)
# data_timestamp = data.loc[:,'rosbagTimestamp']
# data_pressure = data.loc[:,'fluid_pressure']

def ploting(directory):
    namespace = 'br4'
    file = directory + "/" + "_slash_" + namespace + "_slash_distance_sonar.csv"
    data = pd.read_csv(file)
    data_timestamp = data.loc[:,'rosbagTimestamp']
    distance = data.loc[:,'data_offset']
    plt.plot(data_timestamp, distance)
    plt.show()


def main():
    path = sys.argv[1]
    dirs = os.listdir(path)
    for directory in dirs:
        if os.path.isfile(path + "/" + directory):
            continue
        ploting(path + "/" + directory)
        input()
main()