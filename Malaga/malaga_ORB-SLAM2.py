#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: rory_haggart


"""
import h5py    # to read .hdf5 sensor records files
import matplotlib.pyplot as plt
import numpy as np
import re
import sys
import subprocess
import os 
import math
from pyquaternion import Quaternion
import statistics
import bagpy
from bagpy import bagreader
import pandas as pd
import shlex

def main(*args):   
    bagFilePath = "/media/rory_haggart/ENDLESS_BLU/sceneVisibilityInSLAM/Malaga/Malaga/malaga_15/malaga_15.bag"
    
    # get the folder containing all the data
    extractFolder = os.path.abspath(os.path.dirname(bagFilePath))
    extractNumber = extractFolder[-2:]      
    # get the file with the GPS (ground truth) data            
    GPSFile = extractFolder + "/malaga-urban-dataset-extract-{}_all-sensors_GPS.txt".format(extractNumber)
    
    # read the collected GPS data into a list
    with open(GPSFile) as f1:
        GPSDataFile = f1.readlines()
    
    # split the list items by spaces
    GPSData = []
    for line in GPSDataFile:
        GPSData.append(shlex.split(line))

    # get ground truth timestamps
    timestampGT = [float(row[0]) for row in GPSData[1:]]

    poseGT = [None] * 3 # initialise a list for the ground truth pose graphs
    
    # get the position data and extract x, y, z
    poseGT[0] = [float(row[8]) for row in GPSData[1:]]  # x
    poseGT[1] = [float(row[9]) for row in GPSData[1:]]  # y
    poseGT[2] = [float(row[10]) for row in GPSData[1:]] # z
    
    # Get the rotation data so we can compensate the estimation
    IMUFile = extractFolder + "/malaga-urban-dataset-extract-{}_all-sensors_IMU.txt".format(extractNumber)
    
    # read the collected IMU data into a list
    with open(IMUFile) as f1:
        IMUDataFile = f1.readlines()
    
    # split the list items by spaces
    IMUData = []
    for line in IMUDataFile:
        IMUData.append(shlex.split(line))

    # get ground truth timestamps
    timestampIMU = [float(row[0]) for row in IMUData[1:]]

    bearingGT = [None] * 3 # initialise a list for the ground truth bearing
    
    # get the rotation information
    bearingGT[0] = [float(row[12]) for row in IMUData[1:]]  # x
    bearingGT[1] = [float(row[11]) for row in IMUData[1:]]  # y
    bearingGT[2] = [float(row[10]) for row in IMUData[1:]]  # z
    
     
    
    printStatus("Getting Estimate...")
    # ORB-SLAM3 SVE writes the estimated visibility parameters to this .txt file
    trajectoryEstimateFile = input("Paste the full path to the KFT.txt:\n")
    estimate = getEstimate(trajectoryEstimateFile)
    if not isinstance(estimate, pd.DataFrame):
        printStatus("No Output Recorded. Exiting.")
        # e.g. if track never started
        return(-1, -1, -1)
    
    estimateStartTime = estimate.Timestamp.iloc[0]
    estimateEndTime = estimate.Timestamp.iloc[-1]
    
    
    
    
    # ORB-SLAM2 SVE writes the estimated visibility parameters to this .txt file
    SVEPath = input("Paste the full path to the SVE.txt:\n")
    
    printStatus("Getting Scene Visibility Estimation...")
    SVE = getSceneVisibilityEstimate(SVEPath)
    
    # get the visibility timestamps that correspond to the start/end of the estimation
    startIdx = SVE.Timestamp.tolist().index(min(SVE.Timestamp, key=lambda x:abs(x-estimate.Timestamp[0])))
    endIdx = SVE.Timestamp.tolist().index(min(SVE.Timestamp, key=lambda x:abs(x-estimate.Timestamp.iloc[-1])))

    untrackedTime = 0
    for i in range(1, len(SVE)):
        if SVE.c.iloc[i] == 0:
            untrackedTime += (SVE.Timestamp.iloc[i] - SVE.Timestamp.iloc[i-1])
            
    SVE = SVE.truncate(startIdx, endIdx)   
    
    SVE.index = [a - SVE.index[0] for a in SVE.index]

    printStatus("Plotting Results...")
    
    ### PLOTS ###
    # TODO: move plots into their own functions and neaten up
    # plot a 3D representation of the trajectory ground truth and estimate
    fontSize = 15
    
    trajAx = plt.axes(projection='3d')
    trajAx.plot3D(estimate.x, estimate.y, estimate.z, 'blue', label='EST')
    trajAx.view_init(40, 45)
    trajAx.set_xlim([min(estimate.x), max(estimate.x)])
    trajAx.set_xlim([min(estimate.y), max(estimate.y)])
    trajAx.set_xlim([min(estimate.z), max(estimate.z)])
    trajAx.set_xlabel('X (m)', fontsize=fontSize, labelpad=10)
    trajAx.set_ylabel('Y (m)', fontsize=fontSize, labelpad=10)
    trajAx.set_zlabel('Z (m)', fontsize=fontSize, labelpad=10)
    trajAx.dist = 11
    plt.show()
    fig = trajAx.get_figure()
    fig.savefig("Plot_3D.eps",format='eps') 
    
    # 3x2 grid of subplots for pose time series comparison
    fig, axs = plt.subplots(3, 1)   

    # plot the pose graph ground truths 
    for plotIdx in range(3):
        axs[plotIdx].plot(estimate.Timestamp, estimate.iloc[:,(plotIdx % 3) + 1], 'blue', label='EST')
        axs[plotIdx].grid()

    # set the various titles and axis labels
    #axs[0][0].set(title="Position Ground Truths", ylabel="X Position (m)")
    #axs[1][1].set(title="Position Estimates")
    axs[0].set_ylabel("X (m)", fontsize=fontSize)
    axs[1].set_ylabel("Y (m)", fontsize=fontSize)
    axs[2].set_ylabel("Z (m)", fontsize=fontSize)
    axs[2].set_xlabel("Time (s)", fontsize=fontSize)
    
    axs[0].get_yaxis().set_label_coords(-0.12,0.5)
    axs[1].get_yaxis().set_label_coords(-0.12,0.5)
    axs[2].get_yaxis().set_label_coords(-0.12,0.5)
    
    plt.subplots_adjust(left=0.2, hspace = 0.1)
    
    # turn off the tick labels for non-edge plots to avoid clipping
    axs[0].set_xticklabels([])
    axs[1].set_xticklabels([])
    
    plt.show()
    fig.savefig("Plot_separate.eps",format='eps') 
    
    # plot the change in visibility over time
    fig3 = plt.figure(figsize=(15,10))
    plt.subplots_adjust(wspace = 0.35)
    fontSize = 40
    
    ax1 = fig3.add_subplot(2,2,1)
    ax2 = fig3.add_subplot(2,2,3)
    
    ax3 = fig3.add_subplot(3,2,2)
    ax4 = fig3.add_subplot(3,2,4)
    ax5 = fig3.add_subplot(3,2,6)

    #ax1.plot(estimate.Timestamp, errList, color='black', linewidth='4')
    ax1.grid()
    ax1.get_yaxis().set_label_coords(-0.17,0.5)
    ax1.set_ylabel("|Error| (m)", fontsize=fontSize)
    ax1.set_xticklabels([])
    ax1.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax1.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    ax2.plot(SVE.Timestamp, SVE.SVE, label='SVE', color='black', linewidth='4')
    ax2.set_ylabel("SVE", fontsize=fontSize)
    ax2.get_yaxis().set_label_coords(-0.17,0.5)
    ax2.set_ylim([0,1.2])
    ax2.grid()
    ax2.set_xlabel("Time (s)", fontsize=fontSize)
    ax2.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax2.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    ax3.plot(SVE.Timestamp, SVE.a, label='a', color='black', linewidth='4')
    ax3.set_xticklabels([])
    h = ax3.set_ylabel("a", fontsize = fontSize)
    h.set_rotation(0)
    ax3.get_yaxis().set_label_coords(-0.22,0.5)
    ax3.grid()
    ax3.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax3.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    ax4.plot(SVE.Timestamp, SVE.b, label='b', color='black', linewidth='4')
    ax4.set_xticklabels([])
    h = ax4.set_ylabel("b", fontsize = fontSize)
    h.set_rotation(0)
    ax4.get_yaxis().set_label_coords(-0.22,0.5)
    ax4.grid()
    ax4.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax4.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    ax5.plot(SVE.Timestamp, SVE.c, label='c', color='black', linewidth='4')
    ax5.set_xlabel("Time(s)", fontsize = fontSize)
    h = ax5.set_ylabel("c", fontsize = fontSize)
    h.set_rotation(0)
    ax5.get_yaxis().set_label_coords(-0.22,0.5)
    ax5.grid()
    ax5.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax5.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    plt.show()
    
    fig3.savefig("Visibility.eps",format='eps') 

    meanVis = statistics.mean(SVE.SVE)

    meanSVEStats = [statistics.mean(SVE.iloc[:,i]) for i in range(1,5)]
    
    printStatus("\n\n")
    print("Mean Visibility: {}".format(meanSVEStats))
    # TODO: print tracking start, end, total duration (%)


def getEstimate(trajectoryEstimateFile):
    
    ### EXTRACT ESTIMATE DATA
    # open and read the file
    with open(trajectoryEstimateFile, "r") as f2:
        trajectoryOutput = f2.read().splitlines()
    # if the estimator never actually got a good track, exit
    if len(trajectoryOutput) == 0:
        return(-1)
    # split into lists
    trajectoryOutput = [line.split(' ') for line in trajectoryOutput]
    ###
    
    # create an empty dataframe for the ground truth with time, position, orientation columns
    estimateDF = pd.DataFrame(columns=['Timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    
    ### POSITION
    for i in range(1,4):
        # get position data from .hdf5 group and store in dataframe (x,y,z [m])
        estimateDF.iloc[:,i] = [float(row[i]) for row in trajectoryOutput]
    ###
    
    ### ORIENTATION
    for i in range(4,8):
        # get orientation data from .hdf5 group and store in dataframe (x,y,z,w [quaternion])
        estimateDF.iloc[:,i] = [float(row[i]) for row in trajectoryOutput]
    ###

    ### TIMESTAMPS
    # TODO: get start time from .bag file directly
    # subtract the start time of the measurements to begin at 0.0s
    estimateDF.Timestamp = [float(row[0]) - 100000 for row in trajectoryOutput]
    ###
    
    # return the estimate dataframe
    return(estimateDF)

def getSceneVisibilityEstimate(SVEPath):
    
    # create an empty dataframe for the ground truth with time, position, orientation columns
    sveDF = pd.DataFrame(columns=['Timestamp', 'a', 'b', 'c', 'SVE'])
    
    # open the .txt file and grab the lines of output
    with open(SVEPath, "r") as f3:
        SVE_timeSeries = f3.read().splitlines()
    
    # split into lists by comma
    SVE_timeSeries = [line.split(' ') for line in SVE_timeSeries]

    # format (0)timestamp (1)visibility (2)SVE_a (3)SVE_b (4)SVE_c
    sveDF.Timestamp = [(float(row[0]) - 100000) for row in SVE_timeSeries]
    sveDF.SVE = [float(row[1]) for row in SVE_timeSeries]
    sveDF.a = [float(row[2]) for row in SVE_timeSeries]
    sveDF.b = [float(row[3]) for row in SVE_timeSeries]
    sveDF.c = [float(row[4]) for row in SVE_timeSeries]
    
    # reduce to only keyframes to dampen the noise in the values
    #idxList = [np.around(SVE_t,2).tolist().index(a) for a in np.around(SVE_t,2).tolist() if a in np.around(estimate.Timestamp,2).tolist()]
    #idxList = [np.around(sveDF.Timestamp,2).tolist().index(a) for a in np.around(sveDF.Timestamp,2).tolist() if (a/0.1)%1==0]
    #for i in range(5):
    #    sveDF.iloc[:,i] = np.array(sveDF.iloc[:,i])[idxList]
    
    #sveDF = sveDF.dropna()
    #sveDF.index = [a - sveDF.index[0] for a in sveDF.index]
    
    return(sveDF)

def printStatus(statusMsg):
    statusMsg = "\r" + statusMsg + "            "
    print(statusMsg, end = '', flush=True)

if __name__ == "__main__":
    main()