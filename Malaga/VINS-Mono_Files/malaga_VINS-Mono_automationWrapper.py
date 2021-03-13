#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 20 12:05:55 2021

@author: rory_haggart
    This file is acting as the 'setup' wrapper to the automated test process
    The tests in particular deal with the Malaga dataset on VINS_Mono
    This file is used for configurations and then passes the bag path 
        into the 'malaga_VINS-Mono' function to execute the ROS process,
        store the estimated pose output, and compare this to the available
        ground truth. this function should then return a 'score' for each
        axis, where a high score means high deviation from the ground truth.

    # TODO: can we just put the config file at our level and keep away from
        VINS src?
"""
import importlib  
import re
import os

def main():
    malagaOnVINS = importlib.import_module("malaga_VINS-Mono")
    
    """
    ### VINS CONFIGURATION ###
        VINS-Mono has a range of configuration parameters for the particular
            dataset. This includes physical things like the camera calibration,
            and the sensor noises, as well as algorithm parameters like the 
            number of points to select and their minimum distance from one
            another.
        Some of these paramters are pretty fixed (e.g. camera calibration is
            a known thing), but others are less certain and require some level
            of tuning to bring the quality of tracking up
    """
    # the config_file is for configuring VINS_Mono for this particular dataset
    config_file = "/home/rory_haggart/catkin_ws/src/VINS-Mono/config/Malaga/malaga_config.yaml"

    extractNumber = 15  # the number of the Malaga extract to test    

    # dict of configuration parameters for easy iteration and modification
    configParams = {
        "max_cnt": 200, 
        "min_dist": 70,  
        "freq": 20,     
        "equalize": 1,
        "acc_n": 0.08,
        "gyr_n": 0.09,
        "acc_w": 0.00004,
        "gyr_w": 0.000002}

    for item in configParams.items():
        # convert the values to strings and ditch scientific format for floats
        if isinstance(item[1], float):
            configParams[item[0]] = "{:.9f}".format(item[1])
        else:
            configParams[item[0]] = str(item[1])
    
    # read the config file lines into a list
    with open(config_file, "r") as f1:
        config_string = f1.read().splitlines()
    
    # go through each parameter in the dict, and look for that param in each
        # line of the file. if we find it, replace the assigned value with
        # the one we've defined in the dict
    for item in configParams.items():
        for line in config_string:
            if str(item[0]) in line:
                # ': ' matches the colon so we're ignoring comments that might match
                # \d+ is any number of numbers
                # (?:...) matches the decimal point followed by some integers
                # ? matches the expression to the left - the () term - 0 or 1 times
                config_string[config_string.index(line)] = re.sub(': (\d+(?:\.\d+)?)', ': ' + item[1], line)
    
    # write the updated configuration to the config file
    with open(config_file, 'w') as f2:
        for line in config_string:
            f2.write("%s\n" % line)
        
    """
    ### DATASET ROOT ###
      the 'root' directory is the one that contains these python functions, and
          also a 'Malaga' subdirectory which then contains the dataset
          i.e. ROOT
                  VINS-Mono_Files
                      MidAir_automationWrapper.py
                      poseGraph.txt
                      etc.
                  Malaga
                      calibration
                      malaga_01
                      etc.
    """
    root = os.path.abspath(os.path.dirname(os.getcwd()))
    
    bagFilePath = root + "/Malaga" + "/malaga_{}".format(extractNumber) + "/malaga_{}".format(extractNumber) + ".bag"
    
    # TODO: check if the bag file exists, if not, create it
    
    ### RUN TEST AND PRODUCE COMPARISON TO GROUND TRUTH ###
    # TODO: have a 'loop' parameter to execute this n times and take average
        # scores
    rollingSum = [0, 0, 0]
    numLoops = 3
    i = 0
    discontinuityCounter = 0
    while i < numLoops:
        SSE = malagaOnVINS.main(bagFilePath)
        
        # returns [-1, -1, -1] if the test included a discontinuity. we don't
            # like these so we will redo the test
        if SSE == [-1, -1, -1]:
            i -= 1
            discontinuityCounter += 1
        else:
            rollingSum = [(rollingSum[0] + SSE[0]), (rollingSum[1] + SSE[1]), (rollingSum[2] + SSE[2])]
        
        # if we're just getting loads of discontinuities, exit with a high score
        if discontinuityCounter == 3:
            i = 5
            rollingSum = [10000000, 10000000, 10000000] 
        
        i += 1
        
    avgScore = [sumElement / numLoops for sumElement in rollingSum]
    
    print("Average X Score: {:.3f}".format(avgScore[0]))
    print("Average Y Score: {:.3f}".format(avgScore[1]))
    print("Average Z Score: {:.3f}".format(avgScore[2]))
    
    return(avgScore[0] + avgScore[1] + avgScore[2])
    
if __name__ == "__main__":
    main()