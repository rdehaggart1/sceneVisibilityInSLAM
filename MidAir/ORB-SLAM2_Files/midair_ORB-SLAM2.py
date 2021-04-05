#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 14:06:31 2021

@author: rory_haggart

BRIEF:  This script self-contains the full process of running a segment of 
        the midair dataset on the V-SLAM algorithm, ORB-SLAM2. 

        The script requires a single input argument, and this is the full 
        path to a .bag file of a midair segment 
        (e.g. [ROOT]/MidAir/Kite_test/sunny/trajectory_0001_color_down.bag)

The script takes this bag file and passes it to the .sh file which handles
the ORB-SLAM2 process. This includes building the workspace, launching the 
estimator using the config file, running the .bag file, and saving the output
pose estimate to a .txt file.
With this info, it then plots the ground truth (contained in the provided 
.hdf5 sensor files) and the pose estimate for each of the 3 axes. 

TODO:
    Translate the ground truth / estimation frames onto one another
    Ensure portability and readability
    Add a 'start' and 'end' marker onto the x/y graphs?
"""
import h5py    # to read .hdf5 sensor records files
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import re
import sys
import subprocess
import os 
from pyquaternion import Quaternion

def main():
    # the input argument is the path to the bag file that is being run in this test
    bagFilePath = input("Please paste the full path to the .bag file to test:\n")

    shellFile = os.path.abspath(os.getcwd() + "/midair_ORB-SLAM2.sh")
    
    subprocess.run(["chmod", "+x", shellFile], shell=True, executable='/bin/bash')
    # run the shell file with the selected bag file as an input argument
    subprocess.run(shellFile + " " + bagFilePath, shell=True, executable='/bin/bash')
    
    # TODO: if .bag doesn't exist, prompt to create it?
    # TODO: add a 'show graphs' option to the call
    
    # the particular trajectory number
    trajectory = re.search("trajectory_(.*?)_", bagFilePath).group(1)
    
    # define the path to the folder containing our sensor records
    sensorRecordsPath = os.path.abspath(os.path.dirname(bagFilePath))  

    # open sensor_records.hdf5 (for ground truth info)
    f1 = h5py.File(sensorRecordsPath + '/sensor_records.hdf5','r+')   
    
    # get the group for the ground truth records
    groundTruth = f1['trajectory_' + trajectory]['groundtruth']
    
    poseGT = [None] * 3 # initialise a list for the ground truth pose graphs
    
    # get the position data and extract x, y, z
    poseGraphGroundTruth = list(groundTruth['position'])
    poseGT[0] = [row[0] for row in poseGraphGroundTruth] # x
    poseGT[1] = [row[1] for row in poseGraphGroundTruth] # y
    poseGT[2] = [row[2] for row in poseGraphGroundTruth] # z
    
    attitudeGT = [None] * 4 # initialise a list for the ground truth attitude 
    
    # get the attitude data (quaternion)
    attitudeGroundTruth = list(groundTruth['attitude'])
    attitudeGT[0] = [row[0] for row in attitudeGroundTruth] # w
    attitudeGT[1] = [row[1] for row in attitudeGroundTruth] # x
    attitudeGT[2] = [row[2] for row in attitudeGroundTruth] # y
    attitudeGT[3] = [row[3] for row in attitudeGroundTruth] # z
    
    groundTruthRate = 100   # the ground truth data is logged at 100Hz
    
    # create a time series that is the length of the data so we can align 
    # the ground truth with the estimated pose graph
    timestampGT = np.arange(0, len(poseGraphGroundTruth)/groundTruthRate, 1/groundTruthRate).tolist()
    
    # rounding etc. can cause timestamp array to be an element too long
    if len(timestampGT) > len(poseGraphGroundTruth):
        timestampGT = timestampGT[0:len(poseGraphGroundTruth)]
    
    # ORB-SLAM2 writes the pose estimate to this .txt file
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
    timestampEst = [(float(row[0]) - 100000) for row in poseGraphEstimate]
    poseEst[0] = [float(row[3]) for row in poseGraphEstimate]
    poseEst[1] = [float(row[1]) for row in poseGraphEstimate]
    poseEst[2] = [float(row[2]) for row in poseGraphEstimate]
 
    firstTimestampIdx = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[0])))
    lastTimestampIdx = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[-1])))
    
    timestampGT = timestampGT[firstTimestampIdx:lastTimestampIdx]
    poseGT = [line[firstTimestampIdx:lastTimestampIdx] for line in poseGT]
    attitudeGT = [line[firstTimestampIdx:lastTimestampIdx] for line in attitudeGT]
    
    poseGT[0] = [(a - poseGT[0][0]) for a in poseGT[0]]
    poseGT[1] = [(a - poseGT[1][0]) for a in poseGT[1]]
    poseGT[2] = [(a - poseGT[2][0]) for a in poseGT[2]]
    
    initialAttitude = [attitudeGT[el][0] for el in range(4)]
    
    initialQuaternion = Quaternion(initialAttitude)
    
    xyz = [[poseEst[0][a], poseEst[1][a], poseEst[2][a]] for a in range(len(poseEst[0]))]
    
    for i in range(len(xyz)):
        rotatedPose = initialQuaternion.rotate(xyz[i])
        poseEst[0][i] = rotatedPose[0]
        poseEst[1][i] = rotatedPose[1]
        poseEst[2][i] = rotatedPose[2]
    
    scaleFactor = (max(poseGT[0])-min(poseGT[0]))/(max(poseEst[0])-min(poseEst[0]))
    
    poseEst = [[j*scaleFactor for j in i] for i in poseEst]
    
    ### PLOTS ###
    
    trajAx = plt.axes(projection='3d')
    trajAx.plot3D(poseEst[0], poseEst[1], poseEst[2], 'blue', label='EST')
    trajAx.plot3D(poseGT[0], poseGT[1], poseGT[2], 'red', label='GT')
    trajAx.legend()
    plt.show()
    
    # 3x2 grid of subplots for pose time series comparison
    fig, axs = plt.subplots(3, 1)   
    
    # plot the pose graph ground truths 
    for plotIdx in range(3):
        axs[plotIdx].plot(timestampGT, poseGT[plotIdx % 3], 'red', label='GT')
        axs[plotIdx].plot(timestampEst, poseEst[plotIdx % 3], 'blue', label='EST')
        axs[plotIdx].grid()
        axs[plotIdx].set_ylim([min(min(poseGT[plotIdx % 3]), min(poseEst[plotIdx % 3])) -1, max(max(poseGT[plotIdx % 3]), max(poseEst[plotIdx % 3])) + 1])
        axs[plotIdx].legend(loc="upper left")

    # set the various titles and axis labels
    #axs[0][0].set(title="Position Ground Truths", ylabel="X Position (m)")
    #axs[1][1].set(title="Position Estimates")
    axs[0].set(ylabel="X Position (m)")
    axs[1].set(ylabel="Y Position (m)")
    axs[2].set(ylabel="Z Position (m)", xlabel="Time (s)")
    
    # turn off the tick labels for non-edge plots to avoid clipping
    axs[0].set_xticklabels([])
    axs[1].set_xticklabels([])
    
    plt.show()
    
    # 1x2 grid of subplots for x/y pose plot comparisons
    axs2 = plt.axes()   
    
    axs2.plot(poseGT[0], poseGT[1], 'red', label='GT')
    axs2.plot(poseEst[0], poseEst[1], 'blue', label='EST')
    axs2.grid()
    axs2.legend()
    axs2.set(xlabel="X Position (m)", ylabel="Y Position (m)")
    
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
    
    f1.close()  # close the .hdf5 file we opened
    
    discontinuityDetect = np.where(abs(np.diff(np.array(poseEst[2])))>2)[0] + 1
    
    # scores can be artifically low if the estimator reset during tracking, as
        # the position resets to zero, keeping it close to the GT. we should
        # report when this happens so we can redo the test until we get
        # good continuity
    if discontinuityDetect.size != 0:
        SSE = [-1, -1, -1]
    
    return(SSE)

if __name__ == "__main__":
    main()