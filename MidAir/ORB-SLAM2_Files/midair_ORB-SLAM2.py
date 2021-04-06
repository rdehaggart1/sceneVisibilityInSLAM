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
import math
from pyquaternion import Quaternion
from sklearn.metrics import mean_squared_error
from scipy import signal

def main(*args):
    # the input argument is the path to the bag file that is being run in this test
    if len(args) == 0:
        bagFilePath = input("Please paste the full path to the .bag file to test:\n")
    else:
        bagFilePath = args[0]
        
    if not os.path.isfile(bagFilePath):
        sys.exit("Cannot find the provided file")
    
    # we then have a shell file that controls the terminal commands
    shellFile = os.path.abspath(os.getcwd() + "/midair_ORB-SLAM2.sh")
    
    # make the shell file executable
    subprocess.run(["chmod", "+x", shellFile], shell=True, executable='/bin/bash')
    # run the shell file with the selected bag file as an input argument
    subprocess.run(shellFile + " " + bagFilePath, shell=True, executable='/bin/bash')
    
    # TODO: if .bag doesn't exist, prompt to create it?
    
    # get the trajectory number from the provided .bag file name
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
    
    # open the .txt file and grab the lines of output
    with open(poseGraphPath, "r") as f2:
        poseGraphEstimate = f2.read().splitlines()
    
    # if the estimator never actually got a good track, exit
    if len(poseGraphEstimate) == 0:
        return([-1, -1, -1])
    
    # split into lists by comma
    poseGraphEstimate = [line.split(' ') for line in poseGraphEstimate]
    
    poseEst = [None] * 3 # initialise a list for the estimated pose graphs
    
    # format: (0)time, (1)x, (2)y, (3)z, (4)qx, (5)qy, (6)qz, (7)qw
    # NOTE: x,y,z axes are different for MidAir and ORB-SLAM2, so the
        # pose estimate axes are transferred to the MidAir convention
        # midair: x,y,z = forward, rightward, downward
        # orbslam: x,y,z = rightward, downward, forward
    timestampEst = [(float(row[0]) - 100000) for row in poseGraphEstimate]
    poseEst[0] = [float(row[3]) for row in poseGraphEstimate]
    poseEst[1] = [float(row[1]) for row in poseGraphEstimate]
    poseEst[2] = [float(row[2]) for row in poseGraphEstimate]
    
    # get the ground truth timestamps that correspond to the start/end of the estimation
    firstTimestampIdx = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[0])))
    lastTimestampIdx = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[-1])))
    
    # truncate the ground truth to the estimation window
    timestampGT = timestampGT[firstTimestampIdx:lastTimestampIdx]
    poseGT = [line[firstTimestampIdx:lastTimestampIdx] for line in poseGT]
    attitudeGT = [line[firstTimestampIdx:lastTimestampIdx] for line in attitudeGT]
    
    # with the truncation, the ground truth will no longer start at [0,0,0]
        # remove the position bias from the truncated ground truth so that the
        # estimation world frame origin and the ground truth world frame origin were
        # conincident at the time that estimation started
    poseGT[0] = [(a - poseGT[0][0]) for a in poseGT[0]]
    poseGT[1] = [(a - poseGT[1][0]) for a in poseGT[1]]
    poseGT[2] = [(a - poseGT[2][0]) for a in poseGT[2]]
    
    # get the ground truth attitude at the new start time and form quaternion object
    initialAttitude = [attitudeGT[el][0] for el in range(4)]
    initialQuaternion = Quaternion(initialAttitude)
    
    # rearrange the pose est into [x,y,z][x,y,z].. instead of [x,x,..][y,y,..][z,z,..]
    xyz = [[poseEst[0][a], poseEst[1][a], poseEst[2][a]] for a in range(len(poseEst[0]))]
    
    # the origins are now coincident, but any rotations that happened between
        # ground truth start time and estimation start time need to be removed
        # so that at estimation start time the two world frames are aligned
    for i in range(len(xyz)):
        rotatedPose = initialQuaternion.rotate(xyz[i])
        poseEst[0][i] = rotatedPose[0]
        poseEst[1][i] = rotatedPose[1]
        poseEst[2][i] = rotatedPose[2]
    
    # V-SLAM can't get absolute scale. Find the relative scale by looking at
        # the difference in the range of measurements, then scale the estimate
    scaleFactor = (max(poseGT[0])-min(poseGT[0]))/(max(poseEst[0])-min(poseEst[0]))
    poseEst = [[j*scaleFactor for j in i] for i in poseEst]
    
    # ORB-SLAM2 SVE writes the estimated visibility parameters to this .txt file
    SVEPath = os.getcwd() + "/SceneVisibilityEstimation.txt"
    
    # open the .txt file and grab the lines of output
    with open(SVEPath, "r") as f3:
        SVE_timeSeries = f3.read().splitlines()
    
    # split into lists by comma
    SVE_timeSeries = [line.split(' ') for line in SVE_timeSeries]
    
    SVE_stats = [None] * 4    # empty array to store visibility estimation
    
    # format (0)timestamp (1)visibility (2)SVE_a (3)SVE_b (4)SVE_c
    SVE_t = [(float(row[0]) - 100000) for row in SVE_timeSeries]
    SVE_stats[1] = [float(row[2]) for row in SVE_timeSeries]
    SVE_stats[2] = [float(row[3]) for row in SVE_timeSeries]
    SVE_stats[3] = [float(row[4]) for row in SVE_timeSeries]
    
    ka = 0.2
    kb = 0.3
    kc = 0.5
    
    SVE_stats[0] = [(ka*float(row[2])) + (kb*float(row[3])) + (kc*float(row[4])) for row in SVE_timeSeries]
    
    #fs = 25  # Sampling frequency
    #fc = 30  # Cut-off frequency of the filter
    #w = fc / (fs / 2) # Normalize the frequency
    #b, a = signal.butter(5, w, 'low')
    #SVE_stats[0] = signal.filtfilt(b, a, SVE_stats[0])    
    
    # get the visibility timestamps that correspond to the start/end of the estimation
    firstTimestampIdx = SVE_t.index(min(SVE_t, key=lambda x:abs(x-timestampEst[0])))
    lastTimestampIdx = SVE_t.index(min(SVE_t, key=lambda x:abs(x-timestampEst[-1])))
    
    SVE_t = SVE_t[firstTimestampIdx:lastTimestampIdx]
    SVE_stats = [line[firstTimestampIdx:lastTimestampIdx] for line in SVE_stats]
    
    matchedPoseGT = [[None] * len(timestampEst) for _ in range(3)]
    
    # for each estimated position, get the closest ground truth
    for i in range(len(timestampEst)):
        correspondingTimeGT = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[i])))
        matchedPoseGT[0][i] = poseGT[0][correspondingTimeGT]
        matchedPoseGT[1][i] = poseGT[1][correspondingTimeGT]
        matchedPoseGT[2][i] = poseGT[2][correspondingTimeGT]
    
    errList = [None] * len(timestampEst)
    
    # find absolute position error in the estimate
    for i in range(len(matchedPoseGT[0])):
        errList[i] = math.sqrt(pow((poseEst[0][i]-matchedPoseGT[0][i]), 2) + pow((poseEst[1][i]-matchedPoseGT[1][i]), 2) + pow((poseEst[2][i]-matchedPoseGT[2][i]), 2))
    
    ### PLOTS ###
    
    # plot a 3D representation of the trajectory ground truth and estimate
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
    
    # plot the change in visibility over time
    fig3, axs3 = plt.subplots(5,1)
    
    axs3[0].plot(timestampEst, errList)
    axs3[0].grid()
    
    axs3[1].plot(SVE_t, SVE_stats[0], label='SVE')
    axs3[2].plot(SVE_t, SVE_stats[1], label='a')
    axs3[3].plot(SVE_t, SVE_stats[2], label='b')
    axs3[4].plot(SVE_t, SVE_stats[3], label='c')
    axs3[1].grid()
    axs3[2].grid()
    axs3[3].grid()
    axs3[4].grid()
    #axs3[1].legend(loc="upper left")
    
    plt.show()
    
    f1.close()  # close the .hdf5 file we opened
    
    
    rmsx = mean_squared_error(poseEst[0], matchedPoseGT[0], squared=False)
    rmsy = mean_squared_error(poseEst[1], matchedPoseGT[1], squared=False)
    rmsz = mean_squared_error(poseEst[2], matchedPoseGT[2], squared=False)
    
    print("x: {}, y: {}, z: {}".format(rmsx,rmsy,rmsz))
    
    SSE = [0, 0, 0]
    
    return(SSE)

if __name__ == "__main__":
    main()