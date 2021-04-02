#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 09:51:01 2021

@author: rory_haggart

BRIEF:  This script self-contains the full process of running a segment of 
        the malaga dataset on the VI-SLAM algorithm, VINS-Mono. 

        The script requires a single input argument, and this is the full 
        path to a .bag file of a midair segment 
        (e.g. [ROOT]/Malaga/malaga_15/malaga_15.bag)

The script takes this bag file and passes it to the .sh file which handles
the VINS-Mono process. This includes building the workspace, launching the 
estimator using the config file, running the .bag file, and saving the output
pose estimate to a .txt file.
With this info, it then plots the ground truth (contained in the provided 
.hdf5 sensor files) and the pose estimate for each of the 3 axes. 

TODO:
    Ensure portability and readability
    Add input argument to .sh file as the .bag
    Add a 'start' and 'end' marker onto the x/y graphs?
"""
import matplotlib.pyplot as plt
import numpy as np
import re
import sys
import subprocess
import os
import shlex
import math

def main(arg1):
    # the input argument is the path to the bag file that is being run in this test
    bagFilePath = arg1

    shellFile = os.path.abspath(os.getcwd() + "/malaga_VINS-Mono.sh")
    
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
    
    # the .sh script writes the pose estimate to this .txt file
    poseGraphPath = os.path.abspath(os.getcwd() + "/poseGraph.txt")
    
    # open the .txt file and grab the lines of output from rostopic
    with open(poseGraphPath, "r") as f2:
        poseGraphEstimate = f2.read().splitlines()
        
    if len(poseGraphEstimate) == 0:
        return([-1, -1, -1])
    
    # split into lists by comma
    poseGraphEstimate = [line.split(',') for line in poseGraphEstimate]
    # remove the first value as it is just headings
    poseGraphEstimate = poseGraphEstimate[1:-1]
    
    poseEst = [None] * 3 # initialise a list for the estimated pose graphs
    
    # format: (0)time, (1)seq, (2)stamp, (3)frame_id, (4)child_frame_id, (5)x, (6)y, (7)z, (8)orientation.x,orientation.y,orientation.z,orientation.w,covariance0,field.pose.covariance1,field.pose.covariance2,field.pose.covariance3,field.pose.covariance4,field.pose.covariance5,field.pose.covariance6,field.pose.covariance7,field.pose.covariance8,field.pose.covariance9,field.pose.covariance10,field.pose.covariance11,field.pose.covariance12,field.pose.covariance13,field.pose.covariance14,field.pose.covariance15,field.pose.covariance16,field.pose.covariance17,field.pose.covariance18,field.pose.covariance19,field.pose.covariance20,field.pose.covariance21,field.pose.covariance22,field.pose.covariance23,field.pose.covariance24,field.pose.covariance25,field.pose.covariance26,field.pose.covariance27,field.pose.covariance28,field.pose.covariance29,field.pose.covariance30,field.pose.covariance31,field.pose.covariance32,field.pose.covariance33,field.pose.covariance34,field.pose.covariance35,field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z,field.twist.twist.angular.x,field.twist.twist.angular.y,field.twist.twist.angular.z,field.twist.covariance0,field.twist.covariance1,field.twist.covariance2,field.twist.covariance3,field.twist.covariance4,field.twist.covariance5,field.twist.covariance6,field.twist.covariance7,field.twist.covariance8,field.twist.covariance9,field.twist.covariance10,field.twist.covariance11,field.twist.covariance12,field.twist.covariance13,field.twist.covariance14,field.twist.covariance15,field.twist.covariance16,field.twist.covariance17,field.twist.covariance18,field.twist.covariance19,field.twist.covariance20,field.twist.covariance21,field.twist.covariance22,field.twist.covariance23,field.twist.covariance24,field.twist.covariance25,field.twist.covariance26,field.twist.covariance27,field.twist.covariance28,field.twist.covariance29,field.twist.covariance30,field.twist.covariance31,field.twist.covariance32,field.twist.covariance33,field.twist.covariance34,field.twist.covariance35']
    timestampEst = [(float(row[0])*pow(10,-9)) for row in poseGraphEstimate]
    poseEst[0] = [float(row[5]) for row in poseGraphEstimate]
    poseEst[1] = [float(row[6]) for row in poseGraphEstimate]
    poseEst[2] = [float(row[7]) for row in poseGraphEstimate]
    
    """
    # add cushions to the timestamps to keep the est and GT the same length    
    minCushion = np.linspace(0, min(timestampEst), 10).tolist()
    maxCushion = np.linspace(max(timestampEst), max(timestampGT), 10).tolist()
    timestampEst = minCushion + timestampEst + maxCushion
    minCushionSize = len(minCushion)
    maxCushionSize = len(maxCushion)
    
    # fill the start/end cushions with the same start/end pose est values
    for j in range(3):
        poseEst[j] = ([poseEst[j][0]] * minCushionSize) + poseEst[j] + ([poseEst[j][-1]] * maxCushionSize)
    
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
    
    
    evenlySpacedVals = lambda m, n: [i*n//m + n//(2*m) for i in range(m)]
    
    numVals = 500
    
    #timestampGT_idxList = evenlySpacedVals(numVals, len(timestampGT))
    #timestampGT = [timestampGT[i] for i in timestampGT_idxList]
    timestampEst_idxList = evenlySpacedVals(numVals, len(timestampEst))
    timestampEst = [timestampEst[i] for i in timestampEst_idxList]

    for axisIdx in range(3):
        poseEst_idxList = evenlySpacedVals(numVals, len(poseEst[axisIdx]))
        poseEst[axisIdx] = [poseEst[axisIdx][i] for i in poseEst_idxList]

    """
    #timeStartLineStyle = {"x": poseGraphTimestamp[1], "color": "red", "linestyle": "dashed", "linewidth": 0.5}
    
    ### PLOTS ###
    # get the groundtruth heading and timestamp at some index
    """
    indexGT = 60
    headingGT = math.tan(poseGT[1][indexGT]/poseGT[0][indexGT])
    headingTimestampGT = timestampGT[indexGT]
    
    indexEst = timestampEst.index(min(timestampEst, key=lambda x:abs(x-headingTimestampGT)))
    headingEst = math.tan(poseEst[1][indexEst]/poseEst[0][indexEst])
    """
    angle = 45
    #angleRadian = headingGT - headingEst
    angleRadian = angle*(math.pi/180)
    poseEst[0] = [x * math.cos(angleRadian) + y * math.sin(angleRadian) for x,y in zip(poseEst[0], poseEst[1])]
    poseEst[1] = [-x * math.sin(angleRadian) + y * math.cos(angleRadian) for x,y in zip(poseEst[0], poseEst[1])]
    
    
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
    main(sys.argv[1])
    
"""    
def rotate_origin_only(xy, radians):
    #Only rotate a point around the origin (0, 0).
    x, y = xy
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)

    return xx, yy
"""