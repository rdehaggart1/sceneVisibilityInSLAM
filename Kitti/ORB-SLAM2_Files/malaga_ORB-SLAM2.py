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
from pyquaternion import Quaternion
import subprocess
import os
import shlex
import math

def main(arg1):
    # the input argument is the path to the bag file that is being run in this test
    bagFilePath = arg1

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
    
    # the .sh script writes the pose estimate to this .txt file
    poseGraphPath = os.path.abspath(os.getcwd() + "/KeyFrameTrajectory.txt")
    
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
    
    #initialBearing = -0.2
    #poseEst[0] = [x * math.cos(initialBearing) + y * math.sin(initialBearing) for x,y in zip(poseEst[0], poseEst[1])]
    #poseEst[1] = [-x * math.sin(initialBearing) + y * math.cos(initialBearing) for x,y in zip(poseEst[0], poseEst[1])]
    
    
    #TODO: better solution here. what if there's an offset near a zero crossing?
    if(poseEst[0][-1]*poseGT[0][-1] > 0):
        xSign = 1
    else:
        xSign = -1
        
    if(poseEst[1][-1]*poseGT[1][-1] > 0):
        ySign = 1
    else:
        ySign = -1
        
    if(poseEst[2][-1]*poseGT[2][-1] > 0):
        zSign = 1
    else:
        zSign = -1
    
    poseEst[0] = [el*xSign for el in poseEst[0]]
    poseEst[1] = [el*ySign for el in poseEst[1]]
    poseEst[2] = [el*zSign for el in poseEst[2]]

    ### PLOTS ###

    # 3x2 grid of subplots for pose time series comparison
    fig, axs = plt.subplots(3, 2)   
    
    # plot the pose graph ground truths 
    for plotIdx in range(3):
        axs[plotIdx][0].plot(timestampGT, poseGT[plotIdx % 3])
        axs[plotIdx][0].grid()
        axs[plotIdx][0].set_ylim([min(min(poseGT[plotIdx % 3]), min(poseEst[plotIdx % 3])), max(max(poseGT[plotIdx % 3]), max(poseEst[plotIdx % 3]))])
        
    # plot the pose graph estimates
    for plotIdx in range(3):
        axs[plotIdx][1].plot(timestampEst, poseEst[plotIdx % 3])
        axs[plotIdx][1].grid()
        #axs[plotIdx][1].set_xlim(left=0)
        axs[plotIdx][1].set_ylim([min(min(poseGT[plotIdx % 3]), min(poseEst[plotIdx % 3])), max(max(poseGT[plotIdx % 3]), max(poseEst[plotIdx % 3]))])
    
    # set the various titles and axis labels
    axs[0][0].set(title="Position Ground Truths", ylabel="X Position (m)")
    axs[0][1].set(title="Position Estimates")
    axs[1][0].set(ylabel="Y Position (m)")
    axs[2][0].set(ylabel="Z Position (m)", xlabel="Time (s)")
    axs[2][1].set(xlabel="Time (s)")
    
    # turn off the tick labels for non-edge plots to avoid clipping
    axs[0][0].set_xticklabels([])
    axs[0][1].set_xticklabels([])
    axs[1][0].set_xticklabels([])
    axs[1][1].set_xticklabels([])
    
    axs[0][1].set_yticklabels([])
    axs[1][1].set_yticklabels([])
    axs[2][1].set_yticklabels([])
    
    plt.show()
    
    # 1x2 grid of subplots for x/y pose plot comparisons
    fig2, axs2 = plt.subplots(1, 2)   
    
    # TODO: match scales of the GT/Est plots for better comparison
    
    # X/Y pose ground truth
    axs2[0].plot(poseGT[0], poseGT[1])
    axs2[0].grid()
    axs2[0].set(xlabel="X Position (m)", ylabel="Y Position (m)", title="X/Y Position Ground Truth")
    
    # X/Y pose estimate
    axs2[1].plot(poseEst[0], poseEst[1])
    axs2[1].grid()
    axs2[1].set(xlabel="X Position (m)", ylabel="Y Position (m)", title="X/Y Position Estimate")
    
    plt.show()
    
    ### ERROR SCORING ###
    
    # TODO: assess the scoring method
    # TODO: work out what to do with the scores - where to send them for eg
    
    errorList = [None] * 3
    SSE = [None] * 3
    
    for axisIdx in range(3):
        dataMatch = []
        # for each recorded timestep in the estimate
        for time in timestampEst:
            # find the closest match to this timestamp in the ground truth time series
            matched = min(timestampGT, key=lambda x:abs(x-time))
            # and record the data at that timestamp
            dataMatch.append(poseGT[axisIdx][timestampGT.index(matched)])
        
        # find the difference between the two
        errorList[axisIdx] = [abs(dataMatch[i] - poseEst[axisIdx][i]) for i in range(len(dataMatch))]
        # square each term, sum the squares, and divide by the square of the range of ground truth values
        SSE[axisIdx] = sum(map(lambda x:x*x,errorList[axisIdx]))
    
    
    print("Error Cost X: {}".format(SSE[0]))
    print("Error Cost Y: {}".format(SSE[1]))
    print("Error Cost Z: {}".format(SSE[2]))

    discontinuityDetect = np.where(abs(np.diff(np.array(poseEst[2])))>2)[0] + 1
    
    # scores can be artifically low if the estimator reset during tracking, as
        # the position resets to zero, keeping it close to the GT. we should
        # report when this happens so we can redo the test until we get
        # good continuity
    if discontinuityDetect.size != 0:
        SSE = [-1, -1, -1]
    
    return(SSE)

if __name__ == "__main__":
    main("/media/rory_haggart/ENDLESS_BLU/sceneVisibilityInSLAM/Malaga/Malaga/malaga_15/malaga_15.bag")
    
"""    
def rotate_origin_only(xy, radians):
    #Only rotate a point around the origin (0, 0).
    x, y = xy
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)

    return xx, yy
"""