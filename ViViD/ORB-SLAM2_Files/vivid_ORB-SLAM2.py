#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: rory_haggart

BRIEF:  This script self-contains the full process of running a segment of 
        the vivid dataset on the V-SLAM algorithm, ORB-SLAM2. 

        The script requires a single input argument, and this is the full 
        path to a .bag file of a midair segment 
        (e.g. [ROOT]/ViViD/indoor_robust_local.bag)

The script takes this bag file and passes it to the .sh file which handles
the ORB-SLAM2 process. This includes building the workspace, launching the 
estimator using the config file, running the .bag file, and saving the output
pose estimate to a .txt file.
With this info, it then plots the ground truth and the pose estimate for each 
of the 3 axes. 

TODO:
    Translate the ground truth / estimation frames onto one another
    Ensure portability and readability
    Add a 'start' and 'end' marker onto the x/y graphs?
"""
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
    
    # we then have a shell file that controls the terminal commands
    shellFile = os.path.abspath(os.getcwd() + "/vivid_ORB-SLAM2.sh")
    
    # make the shell file executable
    subprocess.run(["chmod", "+x", shellFile], shell=True, executable='/bin/bash')
    # run the shell file with the selected bag file as an input argument
    subprocess.run(shellFile + " " + bagFilePath, shell=True, executable='/bin/bash')
    
    # read the bag file so we can access some of the info
    bagFile = bagreader(bagFilePath)
    
    # the ground truth time is relative to start, but we would like to know
        # the timestamp of the first image to align trajectories
        # to get the actual start time since epoch, look at the timestamps of
        # the .bag file messages. could get /rgb/image but thats a *lot* of
        # data, so query the info topic instead which is published at the same time
    imgmsgs = bagFile.message_by_topic('/rgb/camera_info')
    df = pd.read_csv(imgmsgs)
    
    # we should also rotate the estimation to match the ground truth. 
        # attitude is not contained in ground truth but the on-board IMU does
        # record it, so get the IMU data to extract this info later
    imumsgs = bagFile.message_by_topic('/imu/data')
    df2 = pd.read_csv(imumsgs)
    
    # clean up the generated csv files now that we have the data
    os.remove(imgmsgs)
    os.remove(imumsgs)
    os.rmdir(os.path.dirname(imgmsgs))
    
    # save the time stamp of the first image to subtract from the estimate time
    startTimeIMG = df['header.stamp.secs'][0] + pow(10,-9) * df['header.stamp.nsecs'][0]
    # now store the image timestamps in a list starting from 0.0
    timestampIMG = list(df['header.stamp.secs'] + pow(10,-9) * df['header.stamp.nsecs'] - startTimeIMG)
    
    # get the trajectory name from the provided .bag file name
    trajectory = re.search(".*/(.*?).bag", bagFilePath).group(1)
    # define the ground truth file name
    groundTruthPath = os.path.dirname(os.getcwd()) + "/ViViD/" + trajectory + "_gt.trc"  
    
    # open the .txt file and grab the lines of output
    with open(groundTruthPath, "r") as f1:
        groundTruth = f1.read().splitlines()
    
    poseGT = [None] * 3 # initialise a list for the ground truth pose graphs

    # the GT file includes some headers, data starts after the first blank line
    poseGraphGroundTruth = [line.split() for line in groundTruth[groundTruth.index("") + 1: -1]]
        
    # get the time and position data and extract x, y, z    
    timestampGT = [float(row[1]) for row in poseGraphGroundTruth]
    poseGT[0] = [(float(row[2]) - float(poseGraphGroundTruth[0][2])) for row in poseGraphGroundTruth] # x1
    poseGT[1] = [(float(row[3]) - float(poseGraphGroundTruth[0][3])) for row in poseGraphGroundTruth] # y1
    poseGT[2] = [(float(row[4]) - float(poseGraphGroundTruth[0][4])) for row in poseGraphGroundTruth] # z1
    
    # the ground truth is of the markers on top of the setup
        # we are provided with this extrinsic relation from cam to GT
    #T_cam_gt = np.array([[0.538117713753331,	0.134273164984585,	-0.832105788532870,	-21.1235087691594],
    #            [0.440141675139334,	-0.886700487453141,	0.141554058069359,	-17.1621036951045],
    #            [0.718821696911292,	-0.442417181758784,	-0.536248454854824,	-13.1052654693001],
    #            [0.0, 0.0, 0.0, 1.0]])
    
    rotation_cam_gt = np.array([[0.538117713753331,	0.134273164984585,	-0.832105788532870],
                                [0.440141675139334,	-0.886700487453141,	0.141554058069359],
                                [0.718821696911292,	-0.442417181758784,	-0.536248454854824]])
    
    translation_cam_gt = np.array([[-21.1235087691594],[-17.1621036951045],[-13.1052654693001]])
    
    filler_cam_gt = np.array([0,0,0,1])
    
    T_cam_gt = np.vstack((np.hstack((rotation_cam_gt,translation_cam_gt)), filler_cam_gt))
    
    rotation_cam_gt_transpose = np.matrix.transpose(rotation_cam_gt)
    translation_cam_gt_inverseComponent = -1*(rotation_cam_gt_transpose).dot(translation_cam_gt)
    
    T_cam_gt_inverse = np.vstack((np.hstack((rotation_cam_gt_transpose,translation_cam_gt_inverseComponent)), filler_cam_gt))
    
    # we can then convert the GT from [[x,x..], [y,y..],[z,z..]] to [[x,y,z],[x,y,z],..]
        # this will allow us to transform each coord by the above matrix
    xyz = np.array([[poseGT[0][a], poseGT[1][a], poseGT[2][a], 1] for a in range(len(poseGT[0]))])
    
    for i in range(len(xyz)):
        # multiply the inverse of the cam-gt transformation by each gt point
            # to get the corresponding cam point
        xyzTranslated = T_cam_gt_inverse @ xyz[i]
        # the ground truth is in mm, so convert to m here
        poseGT[0][i] = xyzTranslated[0]/1000
        poseGT[1][i] = xyzTranslated[1]/1000
        poseGT[2][i] = xyzTranslated[2]/1000
    
    poseGT[0] = [a - poseGT[0][0] for a in poseGT[0]]
    poseGT[1] = [a - poseGT[1][0] for a in poseGT[1]]
    poseGT[2] = [a - poseGT[2][0] for a in poseGT[2]]
    
    # ORB-SLAM2 writes the pose estimate to this .txt file
    poseGraphPath = os.getcwd() + "/KeyFrameTrajectory.txt"
    
    # open the .txt file and grab the lines of output
    with open(poseGraphPath, "r") as f2:
        poseGraphEstimate = f2.read().splitlines()
    
    # if the estimator never actually got a good track, exit
    if len(poseGraphEstimate) == 0:
        return(-1,-1)
    
    # split into lists by comma
    poseGraphEstimate = [line.split(' ') for line in poseGraphEstimate]
    
    poseEst = [None] * 3 # initialise a list for the estimated pose graphs
    
    # format: (0)time, (1)x, (2)y, (3)z, (4)qx, (5)qy, (6)qz, (7)qw
        # orbslam: x,y,z = rightward, downward, forward
        # vivid camera frame agrees with this
    timestampEst = [float(row[0]) - startTimeIMG for row in poseGraphEstimate]
    poseEst[0] = [float(row[1]) for row in poseGraphEstimate]
    poseEst[1] = [float(row[2]) for row in poseGraphEstimate]
    poseEst[2] = [float(row[3]) for row in poseGraphEstimate]
    
    if (max(timestampEst) - min(timestampEst))/(max(timestampGT) - min(timestampGT)) < 0.5:
        return(-1,-1)

    #poseEst[0] = [(a - poseEst[0][0]) for a in poseEst[0]]
    #poseEst[1] = [(a - poseEst[1][0]) for a in poseEst[1]]
    #poseEst[2] = [(a - poseEst[2][0]) for a in poseEst[2]]
    
    # get the ground truth timestamps that correspond to the start/end of the estimation
    firstTimestampIdx = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[0])))
    lastTimestampIdx = timestampGT.index(min(timestampGT, key=lambda x:abs(x-timestampEst[-1])))
    
    # truncate the ground truth to the estimation window
    timestampGT = timestampGT[firstTimestampIdx:lastTimestampIdx]
    poseGT = [line[firstTimestampIdx:lastTimestampIdx] for line in poseGT]
    
    # with the truncation, the ground truth will no longer start at [0,0,0]
        # remove the position bias from the truncated ground truth so that the
        # estimation world frame origin and the ground truth world frame origin were
        # conincident at the time that estimation started
    poseGT[0] = [(a - poseGT[0][0]) for a in poseGT[0]]
    poseGT[1] = [(a - poseGT[1][0]) for a in poseGT[1]]
    poseGT[2] = [(a - poseGT[2][0]) for a in poseGT[2]]
    
    attitudeTimestamp = list(df2['header.stamp.secs'] + (pow(10,-9)*df2['header.stamp.nsecs']) - (df2['header.stamp.secs'][0] + (pow(10,-9)*df2['header.stamp.nsecs'][0])))
    firstTimestampIdx = attitudeTimestamp.index(min(attitudeTimestamp, key=lambda x:abs(x-timestampEst[0])))
    lastTimestampIdx = attitudeTimestamp.index(min(attitudeTimestamp, key=lambda x:abs(x-timestampEst[-1])))
    
    initialAttitude = [df2['orientation.w'][firstTimestampIdx], df2['orientation.x'][firstTimestampIdx], df2['orientation.y'][firstTimestampIdx], df2['orientation.z'][firstTimestampIdx]]
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
    SVE_t = [(float(row[0]) - startTimeIMG) for row in SVE_timeSeries]
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
    
    ATE = np.sqrt(np.mean(np.array(errList)**2))

    print("Absolute Trajectory Error: {}".format(ATE))
    
    meanVis = statistics.mean(SVE_stats[0])
    
    print("Mean Visibility: {}".format(meanVis))
    
    return(ATE, meanVis)

if __name__ == "__main__":
    main()