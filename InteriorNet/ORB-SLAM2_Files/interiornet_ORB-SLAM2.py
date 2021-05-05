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

def main(*args):
    
    # the input argument is the path to the bag file that is being run in this test
    if len(args) == 0:
        bagFilePath = input("Please paste the full path to the .bag file to test:\n")
    else:
        bagFilePath = args[0]
        
    if not os.path.isfile(bagFilePath):
        sys.exit("Cannot find the provided file")
    
    # we have a shell file that controls the terminal commands for running this .bag file
    shellFile = os.path.abspath(os.getcwd() + "/interiornet_ORB-SLAM2.sh")
    
    # make the shell file executable
    subprocess.run(["chmod", "+x", shellFile], shell=True, executable='/bin/bash')
    # run the shell file with the selected bag file as an input argument
    subprocess.run(shellFile + " " + bagFilePath, shell=True, executable='/bin/bash')
    
    # get the extract number of the bag file
    extNum = re.search(".*_(.*?).bag", bagFilePath).group(1)
    
    # define the path to the folder containing our sensor records and get ground truth
    groundTruthPath = os.path.dirname(bagFilePath) + "/velocity_angular_{}_{}".format(extNum, extNum)   
    groundTruthFile = groundTruthPath + "/cam0_gt.visim"
    
    # extract the ground truth data from the sensor records file for this trajectory
    timestampGT, poseGT, attitudeGT = getGroundTruth(groundTruthFile)
    
    # ORB-SLAM2 writes the pose estimate to a .txt file after completion
    # extract the timestamped pose estimate from the ORB-SLAM2 output file
        # returns -1, -1 if error
    trajectoryEstimateFile = os.getcwd() + "/KeyFrameTrajectory.txt"
    timestampEst, poseEst = getEstimate(trajectoryEstimateFile)
    if timestampEst == -1:
        # e.g. if track never started
        return(-1, -1)
    if (max(timestampEst) - min(timestampEst))/(max(timestampGT) - min(timestampGT)) < 0.1:
        # if the estimate didn't last longer than half the total time, its bad
        return(-1,-1)
    
    # the estimate won't start/end at bounds of ground truth
        # we should truncate the ground truth data to the window that the 
        # estimate was running for, so comparisons are accurate
    timestampGT, poseGT, attitudeGT = truncateGroundTruth(timestampEst, timestampGT, poseGT, attitudeGT)
    
    # at the time of estimation start, the drone will have translated and rotated
        # relative to ground truth world frame. this means estimation world frame
        # will be offset from ground truth by this position and rotation.
        # this function corrects that and aligns the two frames
    poseEst, poseGT = alignReferenceFrames(poseEst, poseGT, attitudeGT)
    
    # V-SLAM can't get absolute scale. Find the relative scale by looking at
        # the difference in the range of measurements, then scale the estimate
    scaleFactor = (max(poseGT[0])-min(poseGT[0]))/(max(poseEst[0])-min(poseEst[0]))
    poseEst = [[j*scaleFactor for j in i] for i in poseEst]
    
    timestampGT = [a - timestampGT[0] for a in timestampGT]
    timestampEst = [a - timestampEst[0] for a in timestampEst]
    
    # ORB-SLAM2 SVE writes the estimated visibility parameters to this .txt file
    SVEPath = os.getcwd() + "/SceneVisibilityEstimation.txt"
    
    # open the .txt file and grab the lines of output
    with open(SVEPath, "r") as f3:
        SVE_timeSeries = f3.read().splitlines()
    
    # split into lists by comma
    SVE_timeSeries = [line.split(' ') for line in SVE_timeSeries]
    
    SVE_stats = [None] * 4    # empty array to store visibility estimation
    
    # format (0)timestamp (1)visibility (2)SVE_a (3)SVE_b (4)SVE_c
    SVE_t = [(float(row[0])) for row in SVE_timeSeries]
    SVE_stats[1] = [float(row[2]) for row in SVE_timeSeries]
    SVE_stats[2] = [float(row[3]) for row in SVE_timeSeries]
    SVE_stats[3] = [float(row[4]) for row in SVE_timeSeries]
    
    ka = 0.2
    kb = 0.4
    kc = 0.4
    
    # (a)*(kb*b + kc*c)
    SVE_stats[0] = [ka*float(row[2]) + kb*float(row[3]) + kc*float(row[4]) for row in SVE_timeSeries]
    
    # get the visibility timestamps that correspond to the start/end of the estimation
    firstTimestampIdx = SVE_t.index(min(SVE_t, key=lambda x:abs(x-timestampEst[0])))
    lastTimestampIdx = SVE_t.index(min(SVE_t, key=lambda x:abs(x-timestampEst[-1])))
    
    SVE_t = SVE_t[firstTimestampIdx:lastTimestampIdx]
    SVE_stats = [line[firstTimestampIdx:lastTimestampIdx] for line in SVE_stats]
    
    SVE_t = [a - SVE_t[0] for a in SVE_t]
    
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
    # TODO: move plots into their own functions and neaten up
    # plot a 3D representation of the trajectory ground truth and estimate
    trajAx = plt.axes(projection='3d')
    trajAx.plot3D(poseEst[0], poseEst[1], poseEst[2], 'blue', label='EST')
    trajAx.plot3D(poseGT[0], poseGT[1], poseGT[2], 'red', label='GT')
    trajAx.legend()
    plt.show()
    fig = trajAx.get_figure()
    fig.savefig("Plot_3D.eps",format='eps') 
    
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
    fig.savefig("Plot_separate.eps",format='eps') 
    plt.show()
    
    # plot the change in visibility over time
    fig3 = plt.figure(figsize=(15,10))
    plt.subplots_adjust(wspace = 0.35)
    fontSize = 40
    
    ax1 = fig3.add_subplot(2,2,1)
    ax2 = fig3.add_subplot(2,2,3)
    
    ax3 = fig3.add_subplot(3,2,2)
    ax4 = fig3.add_subplot(3,2,4)
    ax5 = fig3.add_subplot(3,2,6)

    ax1.plot(timestampEst, errList, color='black', linewidth='4')
    ax1.grid()
    ax1.get_yaxis().set_label_coords(-0.17,0.5)
    ax1.set_ylabel("|Error| (m)", fontsize=fontSize)
    ax1.set_xticklabels([])
    ax1.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax1.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    ax2.plot(SVE_t, SVE_stats[0], label='SVE', color='black', linewidth='4')
    ax2.set_ylabel("SVE", fontsize=fontSize)
    ax2.get_yaxis().set_label_coords(-0.17,0.5)
    ax2.set_ylim([-1,1.2])
    ax2.grid()
    ax2.set_xlabel("Time (s)", fontsize=fontSize)
    ax2.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax2.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    ax3.plot(SVE_t, SVE_stats[1], label='a', color='black', linewidth='4')
    ax3.set_xticklabels([])
    h = ax3.set_ylabel("a", fontsize = fontSize)
    h.set_rotation(0)
    ax3.get_yaxis().set_label_coords(-0.22,0.5)
    ax3.grid()
    ax3.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax3.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    ax4.plot(SVE_t, SVE_stats[2], label='b', color='black', linewidth='4')
    ax4.set_xticklabels([])
    h = ax4.set_ylabel("b", fontsize = fontSize)
    h.set_rotation(0)
    ax4.get_yaxis().set_label_coords(-0.22,0.5)
    ax4.grid()
    ax4.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax4.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    ax5.plot(SVE_t, SVE_stats[3], label='c', color='black', linewidth='4')
    ax5.set_xlabel("Time(s)", fontsize = fontSize)
    h = ax5.set_ylabel("c", fontsize = fontSize)
    h.set_rotation(0)
    ax5.get_yaxis().set_label_coords(-0.22,0.5)
    ax5.grid()
    ax5.tick_params(axis='both', which='major', labelsize=fontSize-15)
    ax5.tick_params(axis='both', which='minor', labelsize=fontSize-15)
    
    plt.show()
    
    fig3.savefig("Visibility.eps",format='eps') 
    
    ATE = np.sqrt(np.mean(np.array(errList)**2))

    print("Absolute Trajectory Error: {}".format(ATE))
    
    meanVis = statistics.mean(SVE_stats[0])
    
    print("Mean Visibility: {}".format(meanVis))
    
    return(ATE, meanVis)

def getGroundTruth(groundTruthFile):
    # open the ground truth file and grab the lines of output
    with open(groundTruthFile, "r") as f3:
        groundTruth = f3.read().splitlines()
    
    # remove the header line
    groundTruth = groundTruth[1:-1]
    
    # split into lists by comma
    groundTruth = [line.split(',') for line in groundTruth]
    
    ### POSITION
    # initialise an empty list to store the three-axis position values
    position_groundTruth = [None] * 3
    # store as [[x,x,x...], [y,y,y,...], [z,z,z,...]] for easy plotting
    position_groundTruth = [[float(row[i+1]) for row in groundTruth] for i in range(3)]
    ###
    
    ### ORIENTATION
    # initialise an empty list for the ground truth orientation
    orientation_groundTruth = [None] * 4 
    # store as [[w,w,w,...], [x,x,x,...], [y,y,y,...], [z,z,z,...]]
    orientation_groundTruth = [[float(row[i+4]) for row in groundTruth] for i in range(4)]
    ###
    
    ### TIMESTAMPS
    timestamp_groundTruth = [float(row[0])*pow(10,-9) for row in groundTruth]
    ###    
    
    # return the time, pose, orientation
    return(timestamp_groundTruth, position_groundTruth, orientation_groundTruth)

def getEstimate(trajectoryEstimateFile):
    with open(trajectoryEstimateFile, "r") as f2:
        trajectoryOutput = f2.read().splitlines()
        
    # if the estimator never actually got a good track, exit
    if len(trajectoryOutput) == 0:
        return(-1,-1)
    
    # split into lists
    trajectoryOutput = [line.split(' ') for line in trajectoryOutput]
    
    ### POSITION
    # initialise a list for the estimated position
    position_estimate = [None] * 3
    # output .txt format: (0)time, (1)x, (2)y, (3)z, (4)qx, (5)qy, (6)qz, (7)qw
    # NOTE: x,y,z axes are different for InteriorNet and ORB-SLAM2, so the
        # pose estimate axes are transferred to the InteriorNet convention
        # orbslam: x,y,z = rightward, downward, forward
        # interiornet is flipped in y, z (i.e. 180 degree rotation about x)
    position_estimate[0] = [float(row[1]) for row in trajectoryOutput]
    position_estimate[1] = [-1*float(row[2]) for row in trajectoryOutput]
    position_estimate[2] = [-1*float(row[3]) for row in trajectoryOutput]
    ###
    
    ### TIMESTAMPS
    # TODO: get start time from .bag file directly
    # subtract the start time of the measurements to begin at 0.0s
    timestamp_estimate = [float(row[0]) for row in trajectoryOutput]
    ###
    
    # return the time and pose. could also get attitude, but not necessary
    return(timestamp_estimate, position_estimate)

def truncateGroundTruth(timestamp_estimate, timestamp_groundTruth, position_groundTruth, orientation_groundTruth):
    # get the bounds of the trajectory estimate temporally
    estimateStartTime = timestamp_estimate[0]
    estimateEndTime = timestamp_estimate[-1]
    
    # get the indices of the corresponding times in the ground truth
    groundTruthMatchedStart = min(timestamp_groundTruth, key=lambda x:abs(x-estimateStartTime))
    groundTruthMatchedEnd = min(timestamp_groundTruth, key=lambda x:abs(x-estimateEndTime))
    startIdx = timestamp_groundTruth.index(groundTruthMatchedStart)
    endIdx = timestamp_groundTruth.index(groundTruthMatchedEnd)
    
    # truncate the ground truth lists to the defined temporal window
    timestamp_groundTruth_truncated = timestamp_groundTruth[startIdx:endIdx]
    position_groundTruth_truncated = [line[startIdx:endIdx] for line in position_groundTruth]
    orientation_groundTruth_truncated = [line[startIdx:endIdx] for line in orientation_groundTruth]

    return(timestamp_groundTruth_truncated, position_groundTruth_truncated, orientation_groundTruth_truncated)

def alignReferenceFrames(position_estimate, position_groundTruth, orientation_groundTruth):
    ### ORIGIN ALIGNMENT 
    # with the truncation, the ground truth will no longer start at [0,0,0]
        # remove the position bias from the truncated ground truth so that the
        # estimation world frame origin and the ground truth world frame origin are
        # conincident at the time that estimation started
    
    # remove position bias
    position_groundTruth_aligned = [[(a - position_groundTruth[i][0]) for a in position_groundTruth[i]] 
                                    for i in range(3)]
    ###
    
    ### ORIENTATION ALIGNMENT
    # since the estimation starts after finite time t, the estimation world
        # frame will be oriented at some angle offset relative to the ground
        # truth world frame. correct that here
        
    # get the ground truth attitude (quaternion: w,x,y,z) at the estimation start time
    initialAttitude = [orientation_groundTruth[el][0] for el in range(4)]
    initialQuaternion = Quaternion(initialAttitude)
    
    # rearrange the pose est into [x,y,z][x,y,z].. instead of [x,x,..][y,y,..][z,z,..]
    xyz = [[position_estimate[0][a], 
            position_estimate[1][a], 
            position_estimate[2][a]] for a in range(len(position_estimate[0]))]
    
    # create a new position estimate list for the re-oriented data
    position_estimate_aligned = position_estimate.copy()
    
    # rotate each 3-D point by the amount that the estimation is offset from ground truth
    for i in range(len(xyz)):
        rotatedPose = initialQuaternion.rotate(xyz[i])
        position_estimate_aligned[0][i] = rotatedPose[0]
        position_estimate_aligned[1][i] = rotatedPose[1]
        position_estimate_aligned[2][i] = rotatedPose[2]
    ###
    
    return(position_estimate_aligned, position_groundTruth_aligned)
    
if __name__ == "__main__":
    main()