#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 09:51:01 2021

@author: rory_haggart

BRIEF:  This script self-contains the full process of running a segment of 
        the malaga dataset on the V-SLAM algorithm, ORB-SLAM2. 

        The script requires a single input argument, and this is the full 
        path to a .bag file of a malaga segment 
        (e.g. [ROOT]/Malaga/malaga_15/malaga_15.bag)

The script takes this bag file and passes it to the .sh file which handles
the ORB-SLAM2 process. This includes building the workspace, launching the 
estimator using the config file, running the .bag file, and saving the output
pose estimate to a .txt file.
With this info, it then plots the ground truth (contained in the provided 
sensor files) and the pose estimate for each of the 3 axes. 

TODO:
    Ensure portability and readability
    Add input argument to .sh file as the .bag
    Add a 'start' and 'end' marker onto the x/y graphs?
"""
import matplotlib.pyplot as plt
import numpy as np
import re
import sys
import statistics
from pyquaternion import Quaternion
import subprocess
import os
import shlex
import math
import pandas as pd

def main():
    # the input argument is the path to the bag file that is being run in this test
    bagFilePath = input("Please paste the full path to the .bag file to test:\n")

    shellFile = os.path.abspath(os.getcwd() + "/malaga_ORB-SLAM2.sh")
    
    subprocess.run(["chmod", "+x", shellFile], shell=True, executable='/bin/bash')
    # run the shell file passing the selected bag as an input arg
    subprocess.run(shellFile + " " + bagFilePath, shell=True, executable='/bin/bash')
    
    # TODO: if .bag doesn't exist, prompt to create it?
    # TODO: add a 'show graphs' option to the call
    
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
    timestampGT = [val - timestampGT[0] for val in timestampGT]
    
    poseGT = [None] * 3 # initialise a list for the ground truth pose graphs
    
    # get the position data and extract x, y, z
    poseGT[0] = [float(row[10]) for row in GPSData[1:]]  # x
    poseGT[1] = [-1*float(row[9]) for row in GPSData[1:]]  # y
    poseGT[2] = [-1*float(row[8]) for row in GPSData[1:]] # z
    
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
    bearingGT[0] = [float(row[11]) for row in IMUData[1:]]  # x
    bearingGT[1] = [float(row[10]) for row in IMUData[1:]]  # y
    bearingGT[2] = [float(row[12]) for row in IMUData[1:]]  # z
    
    # the .sh script writes the pose estimate to this .txt file
    poseGraphPath = os.getcwd() + "/KeyFrameTrajectory.txt"
    
    # open the .txt file and grab the lines of output from rostopic
    with open(poseGraphPath, "r") as f2:
        poseGraphEstimate = f2.read().splitlines()
        
    if len(poseGraphEstimate) == 0:
        return([-1, -1, -1])
    
    # split into lists by comma
    poseGraphEstimate = [line.split(' ') for line in poseGraphEstimate]
    
    poseEst = [None] * 3 # initialise a list for the estimated pose graphs
    
    # format: (0)time, (1)x, (2)y, (3)z, (4)qx, (5)qy, (6)qz, (7)qw
    timestampEst = [float(row[0]) for row in poseGraphEstimate]
    timestampEst = [val - timestampEst[0] for val in timestampEst]
    poseEst[0] = [float(row[1]) for row in poseGraphEstimate]
    poseEst[1] = [float(row[2]) for row in poseGraphEstimate]
    poseEst[2] = [float(row[3]) for row in poseGraphEstimate]
    
    # find corresponding matches in the ground truth for the start and end of the estimation portion
    GTStartIdx = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[0])))
    GTEndIdx = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[-1])))
    
    # truncate ground truth to only the portion that was tracked
    timestampGT = timestampGT[GTStartIdx:GTEndIdx]
    poseGT = [line[GTStartIdx:GTEndIdx] for line in poseGT]
    
    poseGT[0] = [(a - poseGT[0][0]) for a in poseGT[0]]
    poseGT[1] = [(a - poseGT[1][0]) for a in poseGT[1]]
    poseGT[2] = [(a - poseGT[2][0]) for a in poseGT[2]]
    
    IMUStartIdx = timestampIMU.index(min(timestampIMU, key=lambda x:abs(x-timestampEst[0])))
    initialBearing = bearingGT[2][IMUStartIdx]# - bearingGT[2][0]
    
    initialQuaternion = Quaternion(axis=[0,0,1], angle=initialBearing)
    
    xyz = [[poseEst[0][a], poseEst[1][a], poseEst[2][a]] for a in range(len(poseEst[0]))]
    
    for i in range(len(xyz)):
        rotatedPose = initialQuaternion.rotate(xyz[i])
        poseEst[0][i] = rotatedPose[0]
        poseEst[1][i] = rotatedPose[1]
        poseEst[2][i] = rotatedPose[2]
        
    poseEst[0] = [a * (max(poseGT[0])-min(poseGT[0]))/(max(poseEst[0])-min(poseEst[0])) for a in poseEst[0]]
    poseEst[1] = [a * (max(poseGT[1])-min(poseGT[1]))/(max(poseEst[1])-min(poseEst[1])) for a in poseEst[1]]
    
    printStatus("Getting Scene Visibility Estimation...")
    SVEPath = os.getcwd() + "/SceneVisibilityEstimation.txt"
    SVE = getSceneVisibilityEstimate(SVEPath)
    
    # get the visibility timestamps that correspond to the start/end of the estimation
    startIdx = SVE.Timestamp.tolist().index(min(SVE.Timestamp, key=lambda x:abs(x-timestampEst[0])))
    endIdx = SVE.Timestamp.tolist().index(min(SVE.Timestamp, key=lambda x:abs(x-timestampEst[-1])))

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
    trajAx.plot3D(poseEst[0], poseEst[1], poseEst[2], 'blue', label='EST')
    trajAx.plot3D(poseGT[0], poseGT[1], poseGT[2], 'red', label='GT')
    trajAx.view_init(40, 45)
    trajAx.set_xlabel('X (m)', fontsize=fontSize, labelpad=10)
    trajAx.set_ylabel('Y (m)', fontsize=fontSize, labelpad=10)
    trajAx.set_zlabel('Z (m)', fontsize=fontSize, labelpad=10)
    trajAx.dist = 11
    plt.legend()
    plt.show()
    fig = trajAx.get_figure()
    fig.savefig("Plot_3D.eps",format='eps') 
    
    # 3x2 grid of subplots for pose time series comparison
    fig, axs = plt.subplots(3, 1)   

    # plot the pose graph ground truths 
    for plotIdx in range(3):
        axs[plotIdx].plot(timestampEst, poseEst[plotIdx], 'blue', label='EST')
        axs[plotIdx].plot(timestampGT, poseGT[plotIdx], 'red', label='GT')
        axs[plotIdx].grid()
        axs[plotIdx].legend()
        
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
    
    #ax1 = fig3.add_subplot(1,2,1)
    ax2 = fig3.add_subplot(1,2,1)
    
    ax3 = fig3.add_subplot(3,2,2)
    ax4 = fig3.add_subplot(3,2,4)
    ax5 = fig3.add_subplot(3,2,6)

    #ax1.plot(estimate.Timestamp, errList, color='black', linewidth='4')
    #ax1.grid()
    #ax1.get_yaxis().set_label_coords(-0.17,0.5)
    #ax1.set_ylabel("|Error| (m)", fontsize=fontSize)
    #ax1.set_xticklabels([])
    #ax1.tick_params(axis='both', which='major', labelsize=fontSize-15)
    #ax1.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
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

def getSceneVisibilityEstimate(SVEPath):
    
    # create an empty dataframe for the ground truth with time, position, orientation columns
    sveDF = pd.DataFrame(columns=['Timestamp', 'a', 'b', 'c', 'SVE'])
    
    # open the .txt file and grab the lines of output
    with open(SVEPath, "r") as f3:
        SVE_timeSeries = f3.read().splitlines()
    
    # split into lists by comma
    SVE_timeSeries = [line.split(' ') for line in SVE_timeSeries]

    # format (0)timestamp (1)visibility (2)SVE_a (3)SVE_b (4)SVE_c
    sveDF.Timestamp = [(float(row[0]) - float(SVE_timeSeries[0][0])) for row in SVE_timeSeries]
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
    
"""    
def rotate_origin_only(xy, radians):
    #Only rotate a point around the origin (0, 0).
    x, y = xy
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)

    return xx, yy
"""