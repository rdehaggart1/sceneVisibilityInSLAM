#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 20 12:05:55 2021

@author: rory_haggart
    This file is acting as the 'setup' wrapper to the automated test process
    The tests in particular deal with the MidAir dataset on VINS_Mono
    This file chooses environment, condition, trajectory, camera, and then
        passes into the 'runAndCompare' function to execute the ROS process,
        store the estimated pose output, and compare this to the available
        ground truth. this function should then return a 'score' for each
        axis, where a high score means high deviation from the ground truth.
    This file can then also act as a method for optimisation. Using Dakota
        or similar, this file should be able to modify the various VINS
        parameters, and run some number of tests with each set and take
        an average score. The parameters should be tuned within specified 
        ranges, and the score minimised.
"""
import importlib
import re
from lmfit import Minimizer, Parameters, fit_report, minimize

def main():
    midairOnORB = importlib.import_module("midair_ORB-SLAM2")
    """
    # the config_file is for configuring VINS_Mono for this particular dataset
    config_file = "/home/rory_haggart/catkin_ws/src/VINS-Mono/config/MidAir/midair_config.yaml"

    
    # dict of configuration parameters for easy iteration and modification
    configParams = {
        "max_cnt": 700, 
        "min_dist": 50,  
        "freq": 20,     
        "equalize": 1,
        "acc_n": 0.08,
        "gyr_n": 0.004,
        "acc_w": 0.00004,
        "gyr_w": 2.0e-6}
    #p.valuesdict()

    
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
        
    
    ### DATASET ROOT ###
      the 'root' directory is the one that contains these python functions, and
          also a 'midair' subdirectory which then contains the dataset
          i.e. ROOT
                 MidAir_automationWrapper.py
                 poseGraph.txt
                 etc.
                 MidAir
                   Kite_test
                   Kite_training
                   etc.
    """
    root = "/media/rory_haggart/ENDLESS_BLU/sceneVisibilityInSLAM/MidAir"
    
    """
    ### TEST CONDITIONS ###    
      the environment to run 
          [Kite_test, Kite_training, PLE_test, PLE_training, VO_test]
      the condition (weather / season)
          Kite environment: [cloudy, foggy, sunny, sunset]
          PLE environment: [fall, spring winter]
          VO environment: [foggy, sunny, sunset]
      the number of the trajectory
      the camera to use
          [color_down, color_left, color_right]
    """
    
    # TODO: loop through lists of conditions
    
    environment = "Kite_training" 
    condition = "sunny"
    trajectory = "0021"
    camera = "color_left"
    
    bagFilePath = root + "/MidAir/" + environment + "/" + condition +  "/trajectory_" + trajectory + "_" + camera + ".bag"
    
    # TODO: check if the bag file exists, if not, create it
    
    ### RUN TEST AND PRODUCE COMPARISON TO GROUND TRUTH ###
    # TODO: have a 'loop' parameter to execute this n times and take average
        # scores
    numLoops = 5
    i = 0
    discontinuityCounter = 0
    
    errList = [[None] * 3 for _ in range(numLoops)]
    
    while i < numLoops:
        SSE = midairOnORB.main(bagFilePath)
        
        # TODO: CHECK FOR BAD TRACK
        if SSE == [-1, -1, -1]:
            i -= 1
            discontinuityCounter += 1
        else:
            errList[i] = SSE
            
        # if we're just getting loads of discontinuities, exit with a high score
        if discontinuityCounter == 3:
            i = 5
        
        i += 1
        
    print(errList)
    
if __name__ == "__main__":
    main()