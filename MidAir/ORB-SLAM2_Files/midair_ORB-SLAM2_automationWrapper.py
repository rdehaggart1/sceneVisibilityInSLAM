#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: rory_haggart
    This file is acting as the 'setup' wrapper to the automated test process
    The tests in particular deal with the MidAir dataset on VINS_Mono
    This file chooses environment, condition, trajectory, camera, and then
        passes into the 'runAndCompare' function to execute the ROS process,
        store the estimated pose output, and compare this to the available
        ground truth. this function should then return a 'score' for each
        axis, where a high score means high deviation from the ground truth.
"""
import importlib
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import re 
import os
import sys 
import pandas as pd
from matplotlib.lines import Line2D
from datetime import datetime

def main():
    # to fix an issue with the error bar caps
    matplotlib.rcParams.update({'errorbar.capsize': 4})
    
    # ask how many different bags will be tested
    numBags = int(input("How many .bag files would you like to test?\n"))
    bagFiles = [None] * numBags
    
    # get the list of bags to be tested
    for i in range(numBags):
        bagFiles[i] = input("Please paste the full path to .bag file {}\n".format(i))
        if not os.path.isfile(bagFiles[i]):
            sys.exit("Cannot find the provided file")
        
    # ask how many times each bag should be tested
    numLoops = int(input("How many times would you like to test each of these .bag files?\n"))
    
    # get the runner script
    midairOnORB = importlib.import_module("midair_ORB-SLAM2") 
    
    results = pd.DataFrame(columns=['condition','trajectory','meanATE','meanSVE','meanA','meanB','meanC'])
    
    conditionList = []
    
    for bagFilePath in bagFiles:
        # get the trajectory number and condition so they can be printed on plots
        trajectory = re.search("trajectory_(.*?)_", bagFilePath).group(1)
        condition = re.search(".*/(.*?)/trajectory", bagFilePath).group(1)
        
        conditionList.append(condition)
        
        # TODO: check if the bag file exists, if not, create it
        
        ### RUN TEST AND PRODUCE COMPARISON TO GROUND TRUTH ###
        i = 0   # loop counter
        faultCounter = 0    # checks if estimation is short or doesn't exist

        
        while i < numLoops:
            # run the test on this file
            ATE, meanVis, meanABC = midairOnORB.main(bagFilePath)
            
            # check for faults
            if ATE == -1:
                i -= 1
                faultCounter += 1
            else:
                # store outputs
                if not pd.isna(results.index.max()):
                    resultIdx = results.index.max() + 1
                else:
                    resultIdx = 0
                results.loc[resultIdx] = [condition, trajectory, ATE, meanVis, meanABC[0], meanABC[1], meanABC[2]]
                
            # if we're just getting loads of faults, exit
            if faultCounter == 10:
                i = numLoops
            
            i += 1
    
    idx = 1
    meanSVEList = []
    sdSVEList = []
    conditionIdxList = []
    for conditionID in conditionList:
        conditionResults = results.loc[results.condition == conditionID]
        meanSVEList.append(np.mean(conditionResults.meanSVE))
        meanSVEList.append(np.mean(conditionResults.meanA))
        meanSVEList.append(np.mean(conditionResults.meanB))
        meanSVEList.append(np.mean(conditionResults.meanC))
        
        sdSVEList.append(np.std(conditionResults.meanSVE))
        sdSVEList.append(np.std(conditionResults.meanA))
        sdSVEList.append(np.std(conditionResults.meanB))
        sdSVEList.append(np.std(conditionResults.meanC))
        
        for j in range(4):
            conditionIdxList.append(idx + (j/10))
        idx+=1
    
    fontSize=12

    fig, ax1 = plt.subplots(1, 1)
    
    colorList = ["#117733", "#CC6677", "#DDCC77", "#88CCEE"]
    
    print("Conditions: {}".format(conditionList))
    print("SVE Means: {}".format(meanSVEList))
    print("SVE Standard Deviations: {}".format(sdSVEList))
    
    for i in range(len(conditionIdxList)):
        markerColor = colorList[i%4]
        ax1.errorbar(conditionIdxList[i], meanSVEList[i], sdSVEList[i], linestyle='None', marker='o',lw=1, fmt=markerColor)  
    
    # plot mean with error bars showing the min and max values
    #ax1.errorbar(conditionList, meanSVEList, sdSVEList, linestyle='None', marker='o',lw=1, fmt='.k')
    ax1.set_title("MidAir, VO_test, Trajectory " + int(trajectory[1:]))
    ax1.set_xlabel('Condition', fontsize=fontSize)
    ax1.set_ylabel('Visibility Estimate', fontsize=fontSize)
    right_side = ax1.spines["right"]
    right_side.set_visible(False)
    top_side = ax1.spines["top"]
    top_side.set_visible(False)
    
    # make x values equal to the integer test numbers
    ax1.set_xticks([a + .15 for a in range(1,len(conditionList) + 1)])
    ax1.set_xticklabels(conditionList)
    # limit y axis to appropriate range
    #ax1.set_yticks(np.arange(0,np.max(scenarios)+1))
    # only horizontal grid lines
    ax1.set_yticks(np.arange(0,1.1,0.1))
    ax1.set_ylim([0,1.1])
    ax1.set_xlim([0.5, len(conditionList) + 0.8])
    ax1.legend([Line2D([0], [0], color=colorList[0], lw=2, label='Line'),
                Line2D([0], [0], color=colorList[1], lw=2, label='Line'),
                Line2D([0], [0], color=colorList[2], lw=2, label='Line'),
                Line2D([0], [0], color=colorList[3], lw=2, label='Line')], ['SVE', 'a', 'b', 'c'])
    
    ax1.grid(axis='y')
    ax1.set_aspect(1)
    # Set tick font size
    for label in (ax1.get_xticklabels() + ax1.get_yticklabels()):
    	label.set_fontsize(fontSize)
    
    now = datetime.now()
    currentTime = now.strftime("%d-%m-%Y_%H-%M-%S")
    
    fig.savefig(os.getcwd() + "/" + currentTime + "_" + trajectory + ".eps",format='eps') 
    
    
if __name__ == "__main__":
    main()