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
import numpy as np
import re 
def main():
    # ask how many different bags will be tested
    numBags = int(input("How many .bag files would you like to test?\n"))
    bagFiles = [None] * numBags
    
    # get the list of bags to be tested
    for i in range(numBags):
        bagFiles[i] = input("Please paste the full path to .bag file {}\n".format(i))
    
    # ask how many times each bag should be tested
    numLoops = int(input("How many times would you like to test each of these .bag files?\n"))
    
    # get the runner script
    midairOnORB = importlib.import_module("midair_ORB-SLAM2")

    for bagFilePath in bagFiles:
        # get the trajectory number and condition so they can be printed on plots
        trajectory = re.search("trajectory_(.*?)_", bagFilePath).group(1)
        condition = re.search(".*/(.*?)/trajectory", bagFilePath).group(1)
        
        # TODO: check if the bag file exists, if not, create it
        
        ### RUN TEST AND PRODUCE COMPARISON TO GROUND TRUTH ###
        i = 0   # loop counter
        faultCounter = 0    # checks if estimation is short or doesn't exist
        
        # lists for absolute trajectory errors and average visibility
        ATEList = [None] * numLoops
        avgVisList = [None] * numLoops
        
        while i < numLoops:
            # run the test on this file
            ATE, meanVis = midairOnORB.main(bagFilePath)
            
            # check for faults
            if ATE == -1:
                i -= 1
                faultCounter += 1
            else:
                # store outputs
                ATEList[i] = ATE
                avgVisList[i] = meanVis
                
            # if we're just getting loads of faults, exit with a high score
            if faultCounter == 3:
                i = 5
            
            i += 1
            
        print(ATEList)
        print(avgVisList)
        
        medianATE = np.median(ATEList)
        
        plt.scatter(ATEList, avgVisList,marker='x')
        plt.plot(np.unique(ATEList), np.poly1d(np.polyfit(ATEList, avgVisList, 1))(np.unique(ATEList)))
        plt.title("Trajectory " + trajectory + ", " + condition + ". Median ATE: " + str(medianATE))
        plt.xlabel("Absolute Trajectory Error (m RMSE)")
        plt.ylabel("Average Visibility Score")
        plt.grid()
        plt.show()
    
    
if __name__ == "__main__":
    main()