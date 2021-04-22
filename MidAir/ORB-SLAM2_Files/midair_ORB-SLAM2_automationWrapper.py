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
    
    meanATEList = []
    meanSVEList = []
    sdATEList = []
    sdSVEList = []
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
        
        # lists for absolute trajectory errors and average visibility
        ATEList = [None] * numLoops
        avgVisList = [None] * numLoops
        avgABCList = [[None,None,None]] * numLoops
        
        while i < numLoops:
            # run the test on this file
            ATE, meanVis, meanABC = midairOnORB.main(bagFilePath)
            
            # check for faults
            if ATE == -1:
                i -= 1
                faultCounter += 1
            else:
                # store outputs
                ATEList[i] = ATE
                avgVisList[i] = meanVis
                avgABCList[i] = meanABC
                
            # if we're just getting loads of faults, exit
            if faultCounter == 10:
                i = numLoops
            
            i += 1
        
        ATEList = [val for val in ATEList if val]
        avgVisList = [val for val in avgVisList if val]
        #avgABCList = [val for val in avgVisList if val]
        
        print(ATEList)
        print(avgVisList)
        #print(avgABCList)
        
        meanATE = np.mean(ATEList)
        meanSVE = np.mean(avgVisList)
        stdDevSVE = np.std(avgVisList)
        stdDevATE = np.std(ATEList)
        
        meanATEList.append(meanATE)
        meanSVEList.append(meanSVE)
        sdATEList.append(stdDevATE)
        sdSVEList.append(stdDevSVE)
        
        plt.scatter(ATEList, avgVisList,marker='x')
        plt.plot(np.unique(ATEList), np.poly1d(np.polyfit(ATEList, avgVisList, 1))(np.unique(ATEList)))
        plt.title("Trajectory " + trajectory + ", " + condition + ". Mean ATE: " + str(meanATE))
        plt.xlabel("Absolute Trajectory Error (m RMSE)")
        plt.ylabel("Average Visibility Score")
        plt.grid()
        plt.show()
    
    fontSize=12
    
    fig, ax1 = plt.subplots(1, 1)
    
    # plot mean with error bars showing the min and max values
    ax1.errorbar(conditionList, meanATEList, sdATEList, linestyle='None', marker='o',lw=1, fmt='.k')
    ax1.set_title("MidAir, Trajectory " + trajectory)
    ax1.set_xlabel('Condition', fontsize=fontSize)
    ax1.set_ylabel('Absolute Trajectory Error', fontsize=fontSize)
    right_side = ax1.spines["right"]
    right_side.set_visible(False)
    top_side = ax1.spines["top"]
    top_side.set_visible(False)

    # make x values equal to the integer test numbers
    ax1.set_xticks(conditionList)
    # limit y axis to appropriate range
    #ax1.set_yticks(np.arange(0,np.max(scenarios)+1))
    # only horizontal grid lines
    ax1.grid()
    # Set tick font size
    for label in (ax1.get_xticklabels() + ax1.get_yticklabels()):
    	label.set_fontsize(fontSize)
    
    
    print(meanATEList)
    print(meanSVEList)
    
if __name__ == "__main__":
    main()