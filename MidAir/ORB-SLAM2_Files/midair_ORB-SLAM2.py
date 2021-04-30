#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
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

def main(*args):
    
    # the input argument is the path to the bag file that is being run in this test
    if len(args) == 0:
        bagFilePath = input("Please paste the full path to the .bag file to test:\n")
    else:
        bagFilePath = args[0]
        
    if not os.path.isfile(bagFilePath):
        sys.exit("Cannot find the provided file")
    
    # we have a shell file that controls the terminal commands for running this .bag file
    shellFile = os.path.abspath(os.getcwd() + "/midair_ORB-SLAM2.sh")
    print("\n")
    printStatus("Execution Begin...")
    
    # make the shell file executable
    subprocess.run(["chmod", "+x", shellFile], shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # run the shell file with the selected bag file as an input argument
    subprocess.run(shellFile + " " + bagFilePath, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    printStatus("Execution End...")
    
    # get the trajectory number from the provided .bag file name
    trajectory = re.search("trajectory_(.*?)_", bagFilePath).group(1)
    camera = re.search("color_(.*?).bag", bagFilePath).group(1)

    condition = re.search(".*/(.*?)/trajectory", bagFilePath).group(1)
    environment = re.search(".*/MidAir/(.*?)/{}".format(condition), bagFilePath).group(1)
    
    # define the path to the folder containing our sensor records and get ground truth
    sensorRecordsPath = os.path.abspath(os.path.dirname(bagFilePath))  
    sensorRecords = h5py.File(sensorRecordsPath + '/sensor_records.hdf5','r+')   
    
    printStatus("Getting Ground Truth...")
    # extract the ground truth data from the sensor records file for this trajectory
    groundTruth = getGroundTruth(sensorRecords, trajectory)
    
    groundTruthStartTime = groundTruth.Timestamp.iloc[0]
    groundTruthEndTime = groundTruth.Timestamp.iloc[-1]
    
    printStatus("Getting Estimate...")
    # ORB-SLAM2 writes the pose estimate to a .txt file after completion
    # extract the timestamped pose estimate from the ORB-SLAM2 output file
        # returns -1, -1 if error
    trajectoryEstimateFile = os.getcwd() + "/KeyFrameTrajectory.txt"
    estimate = getEstimate(trajectoryEstimateFile)
    if not isinstance(estimate, pd.DataFrame):
        printStatus("No Output Recorded. Exiting.")
        # e.g. if track never started
        return(-1, -1, -1)
    if (max(estimate.Timestamp) - min(estimate.Timestamp))/(max(groundTruth.Timestamp) - min(groundTruth.Timestamp)) < 0.5:
        printStatus("Output Not Maintained For Long Enough. Exiting.")
        # if the estimate didn't last longer than half the total time, its bad
        return(-1, -1, -1)
    
    estimateStartTime = estimate.Timestamp.iloc[0]
    estimateEndTime = estimate.Timestamp.iloc[-1]
    
    printStatus("Aligning Ground Truth with Estimate...")
    estimate = matchAxisConventions(camera, estimate)
    
    # the estimate won't start/end at bounds of ground truth
        # we should truncate the ground truth data to the window that the 
        # estimate was running for, so comparisons are accurate
    estimate, groundTruth = truncateGroundTruth(estimate, groundTruth)
    
    # at the time of estimation start, the drone will have translated and rotated
        # relative to ground truth world frame. this means estimation world frame
        # will be offset from ground truth by this position and rotation.
        # this function corrects that and aligns the two frames
    estimate, groundTruth = alignReferenceFrames(estimate, groundTruth)
    
    # V-SLAM can't get absolute scale. Find the relative scale by looking at
        # the difference in the range of measurements, then scale the estimate
    scaleFactor = (max(groundTruth.x)-min(groundTruth.x))/(max(estimate.x)-min(estimate.x))
    for i in range(1,4):
        estimate.iloc[:,i] = [a*scaleFactor for a in estimate.iloc[:,i]]
    
    # ORB-SLAM2 SVE writes the estimated visibility parameters to this .txt file
    SVEPath = os.getcwd() + "/SceneVisibilityEstimation.txt"
    
    printStatus("Getting Scene Visibility Estimation...")
    SVE = getSceneVisibilityEstimate(SVEPath)
    
    # get the visibility timestamps that correspond to the start/end of the estimation
    startIdx = SVE.Timestamp.tolist().index(min(SVE.Timestamp, key=lambda x:abs(x-estimate.Timestamp[0])))
    endIdx = SVE.Timestamp.tolist().index(min(SVE.Timestamp, key=lambda x:abs(x-estimate.Timestamp.iloc[-1])))
    
    #startIdx = groundTruthDF.isin([groundTruthMatchedStart])['Timestamp'][groundTruthDF.isin([groundTruthMatchedStart])['Timestamp'] == True].index[0]
    #endIdx = groundTruthDF.isin([groundTruthMatchedEnd])['Timestamp'][groundTruthDF.isin([groundTruthMatchedEnd])['Timestamp'] == True].index[0]
 
    untrackedTime = 0
    for i in range(1, len(SVE)):
        if SVE.c.iloc[i] == 0:
            untrackedTime += (SVE.Timestamp.iloc[i] - SVE.Timestamp.iloc[i-1])
            
    percentageTracked = 100 * (1 - (untrackedTime / (groundTruthEndTime - groundTruthStartTime)))
 
    SVE = SVE.truncate(startIdx, endIdx)   
    
    SVE.index = [a - SVE.index[0] for a in SVE.index]
    
    printStatus("Calculating Trajectory Error...")
    
    matchedPoseGT = [[None] * len(estimate['Timestamp']) for _ in range(3)]
    
    ### ERROR CALCULATIONS
    # for each estimated position, get the closest ground truth
    for i in range(len(estimate.Timestamp)):
        minTimeDiff = min(groundTruth.Timestamp, key=lambda x:abs(x-estimate.Timestamp[i]))
        idx = groundTruth.isin([minTimeDiff])['Timestamp'][groundTruth.isin([minTimeDiff])['Timestamp'] == True].index[0]
        correspondingTimeGT = groundTruth.Timestamp.index[idx]
        matchedPoseGT[0][i] = groundTruth.x[correspondingTimeGT]
        matchedPoseGT[1][i] = groundTruth.y[correspondingTimeGT]
        matchedPoseGT[2][i] = groundTruth.z[correspondingTimeGT]
    
    errList = [None] * len(estimate)
    
    # find absolute position error in the estimate
    for i in range(len(matchedPoseGT[0])):
        errList[i] = math.sqrt(pow((estimate.x[i]-matchedPoseGT[0][i]), 2) + pow((estimate.y[i]-matchedPoseGT[1][i]), 2) + pow((estimate.z[i]-matchedPoseGT[2][i]), 2))
    
    printStatus("Plotting Results...")
    
    ### PLOTS ###
    # TODO: move plots into their own functions and neaten up
    # plot a 3D representation of the trajectory ground truth and estimate
    fontSize = 15
    
    trajAx = plt.axes(projection='3d')
    trajAx.plot3D(estimate.x, estimate.y, estimate.z, 'blue', label='EST')
    trajAx.plot3D(groundTruth.x, groundTruth.y, groundTruth.z, 'red', label='GT')
    trajAx.legend()
    trajAx.view_init(40, 45)
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
        axs[plotIdx].plot(groundTruth.Timestamp, groundTruth.iloc[:,(plotIdx % 3) + 1], 'red', label='GT')
        axs[plotIdx].plot(estimate.Timestamp, estimate.iloc[:,(plotIdx % 3) + 1], 'blue', label='EST')
        axs[plotIdx].grid()
        axs[plotIdx].set_ylim([min(min(groundTruth.iloc[:,(plotIdx % 3) + 1]), min(estimate.iloc[:,(plotIdx % 3) + 1])) -1, max(max(groundTruth.iloc[:,(plotIdx % 3) + 1]), max(estimate.iloc[:,(plotIdx % 3) + 1])) + 1])
        axs[plotIdx].legend(loc='upper left')

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

    ax1.plot(estimate.Timestamp, errList, color='black', linewidth='4')
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
    
    sensorRecords.close()  # close the .hdf5 file we opened
    
    ATE = np.sqrt(np.mean(np.array(errList)**2))

    meanVis = statistics.mean(SVE.SVE)

    meanSVEStats = [statistics.mean(SVE.iloc[:,i]) for i in range(1,5)]
    
    with open("results.txt", 'a') as writer:
        writer.write("Env: {}, Traj: {}, Cond: {}, ATE: {:.3f}m, SVE: {:.3f}, SVE_a: {:.3f}, SVE_b: {:.3f}, SVE_c: {:.3f}, %Tr: {}\n".format(environment,trajectory,condition,ATE,meanVis,meanSVEStats[0],meanSVEStats[1],meanSVEStats[2],percentageTracked))
    
    printStatus("\n\n")
    print("Environment: {} | Trajectory: {} | Condition: {} \n".format(environment, trajectory, condition))
    print("Absolute Trajectory Error: {:.3f}m".format(ATE))
    print("Mean Visibility: {:.3f}".format(meanVis))
    print("Estimate Start: {:.2f}s | Estimate End: {:.2f}s | Percentage Tracked: {:.2f}%".format(estimateStartTime, estimateEndTime, percentageTracked))
    # TODO: print tracking start, end, total duration (%)
    
    return(ATE, meanVis, meanSVEStats)

def getGroundTruth(sensorRecordsFile, trajectoryNumber):
    # get the ground truth group from the sensor records file
    groundTruth = sensorRecordsFile['trajectory_' + trajectoryNumber]['groundtruth']
    
    # create an empty dataframe for the ground truth with time, position, orientation columns
    groundTruthDF = pd.DataFrame(columns=['Timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
    
    ### POSITION
    for i in range(1,4):
        # get position data from .hdf5 group and store in dataframe (x,y,z [m])
        groundTruthDF.iloc[:,i] = [row[i-1] for row in list(groundTruth['position'])]
    ###
    
    ### ORIENTATION
    for i in range(4,8):
        # get orientation data from .hdf5 group and store in dataframe (w,x,y,z [quaternion])
        groundTruthDF.iloc[:,i] = [row[i-4] for row in list(groundTruth['attitude'])]
    ###
    
    ### TIMESTAMPS
    # the ground truth is logged at 100Hz for the MidAir dataset
    groundTruthFrequency = 100   
    # no time data is provided in sensor records, so generate based on freq
    timestamp_groundTruth = np.arange(0, 
                                      len(groundTruthDF)/groundTruthFrequency, 
                                      1/groundTruthFrequency).tolist()
    # rounding of np.arrange end point can cause timestamp array to be an element too long
    if len(timestamp_groundTruth) > len(groundTruthDF):
        # clip the timestamp array down to size
        timestamp_groundTruth = timestamp_groundTruth[0:len(groundTruthDF)]
    
    # store the timestamps in the dataframe
    groundTruthDF.Timestamp = timestamp_groundTruth
    ###    

    # return the ground truth dataframe
    return(groundTruthDF)

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

def matchAxisConventions(camera, estimateDF): 
    # NOTE: x,y,z axes are different for MidAir and ORB-SLAM2, so the
        # pose estimate axes are transferred to the MidAir convention
        # midair: x,y,z = forward, rightward, downward
        # orbslam: x,y,z = rightward, downward, forward   
    
    # define a rotation matrix that maps the orbslam output into the appropriate
        # midair convention. when using color left, we want
        # x->y, y->z, z->x for example
    if camera=='left':
        R_orb_midair = np.array([[0,0,1],[1,0,0],[0,1,0]])
    elif camera=='down':
        R_orb_midair = np.array([[0,-1,0],[1,0,0],[0,0,1]])
    else:
        print("Camera Not Supported")
        sys.exit(0)
        
    # rearrange the pose est into [x,y,z][x,y,z].. instead of [x,x,..][y,y,..][z,z,..]
    xyz = [[estimateDF.x[a], 
            estimateDF.y[a], 
            estimateDF.z[a]] for a in range(len(estimateDF))]
    
    for i in range(len(xyz)):
        # apply the rotation to each set of coordinates
        rotatedPose = R_orb_midair @ xyz[i]
        # store the rotation
        for j in range(1,4):
            estimateDF.iloc[:,j][i] = rotatedPose[j-1]

    # return to main
    return estimateDF

def truncateGroundTruth(estimateDF, groundTruthDF):
    # get the bounds of the trajectory estimate temporally
    estimateStartTime = estimateDF.Timestamp[0]
    estimateEndTime = estimateDF.Timestamp.iloc[-1]
    
    # get the indices of the corresponding times in the ground truth
    groundTruthMatchedStart = min(groundTruthDF.Timestamp, key=lambda x:abs(x-estimateStartTime))
    groundTruthMatchedEnd = min(groundTruthDF.Timestamp, key=lambda x:abs(x-estimateEndTime))
    startIdx = groundTruthDF.isin([groundTruthMatchedStart])['Timestamp'][groundTruthDF.isin([groundTruthMatchedStart])['Timestamp'] == True].index[0]
    endIdx = groundTruthDF.isin([groundTruthMatchedEnd])['Timestamp'][groundTruthDF.isin([groundTruthMatchedEnd])['Timestamp'] == True].index[0]
    
    # truncate the ground truth to the defined temporal window
    groundTruthDF = groundTruthDF.truncate(startIdx,endIdx)

    groundTruthDF.index = [a - groundTruthDF.index[0] for a in groundTruthDF.index]
    
    return(estimateDF, groundTruthDF)

def alignReferenceFrames(estimateDF, groundTruthDF):
    ### ORIGIN ALIGNMENT 
    # with the truncation, the ground truth will no longer start at [0,0,0]
        # remove the position bias from the truncated ground truth so that the
        # estimation world frame origin and the ground truth world frame origin are
        # conincident at the time that estimation started
    
    # remove position bias
    for i in range(1,4):
        groundTruthDF.iloc[:,i] = [a - groundTruthDF.iloc[:,i][0] for a in groundTruthDF.iloc[:,i]]
    ###
    
    ### ORIENTATION ALIGNMENT
    # since the estimation starts after finite time t, the estimation world
        # frame will be oriented at some angle offset relative to the ground
        # truth world frame. correct that here
        
    # get the ground truth attitude (quaternion: w,x,y,z) at the estimation start time
    initialAttitude = [groundTruthDF.iloc[:,i][0] for i in range(4,8)]
    initialQuaternion = Quaternion(initialAttitude)
    
    # rearrange the pose est into [x,y,z][x,y,z].. instead of [x,x,..][y,y,..][z,z,..]
    xyz = [[estimateDF.x[a], 
            estimateDF.y[a], 
            estimateDF.z[a]] for a in range(len(estimateDF))]

    # rotate each 3-D point by the amount that the estimation is offset from ground truth
    for i in range(len(xyz)):
        rotatedPose = initialQuaternion.rotate(xyz[i])
        # store the rotation
        for j in range(1,4):
            estimateDF.iloc[:,j][i] = rotatedPose[j-1]
    ###
    
    return(estimateDF, groundTruthDF)

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