#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  5 15:03:17 2021

@author: rory_haggart

BRIEF:  This script allows a quick look at the acceleration data of a given
        segment of the midair dataset 
       
This script looks at the provided .hdf5 sensor files and plots the accelerations
as provided by the accelerometer. this could be useful for assessing general
motion trends or the impact of noise on the measurements.

TODO:
    ensure portability and readability
    add extra functionality for gyro
"""
# to read .hdf5 sensor records files
import h5py    
import os
import math
import matplotlib.pyplot as plt
import numpy as np

environment = 'Kite_test' # the environment of the simulation
condition = 'sunny' # the weather we're interested in
trajectory = '0001' # the particular trajectory number
camera = 'color_down' # the camera that we're using

# define the path to the folder containing our sensor records
sensorRecordsPath = '/media/rory_haggart/ENDLESS_BLU/SLAM_datasets/MidAir/MidAir/' + environment + '/' + condition

f1 = h5py.File(sensorRecordsPath + '/sensor_records.hdf5','r+')   # open sensor_records.hdf5

# list(f1['trajectory_0001'].keys()) to see keys in file

accelerometer = f1['trajectory_' + trajectory]['imu']['accelerometer']
gyroscope = f1['trajectory_' + trajectory]['imu']['gyroscope']

# get the accelerometer data from the sensor records (m/s^2)
accelerometerData = list(accelerometer) 
# get the gyroscope data from the sensor records (rad/s)
gyroscopeData = list(gyroscope) 
# list the relative paths of the images for the selected camera
imagePaths = list(f1['trajectory_' + trajectory]['camera_data'][camera]) 

accelerationGroundTruth = f1['trajectory_' + trajectory]['groundtruth']['acceleration']

timeSeries = np.arange(0, len(accelerationGroundTruth)/100, 1/100).tolist()
for i in range(3):
    xAccelerationTrue = [row[i] for row in accelerationGroundTruth]
    xAcceleration = [row[i] for row in accelerometerData]
    #plt.plot(timeSeries, xAccelerationTrue, color="green")
    plt.plot(timeSeries, list(xAcceleration), color="red")
    plt.show()