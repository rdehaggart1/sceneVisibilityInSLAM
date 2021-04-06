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
"""
def main():    
    
    Parameter descriptions and example values
    maxNumFeatures = 200        # maximum number of features
    minSepFeatures = 90         # minimum separation of adjacent features
    trackingFrequency = 20      # rate at which tracking result is published (Hz)
    equalizeImg = 1             # equalise the image? (0/1)
    accNoiseStdDev = 0.02616    # standard deviation of accelerometer noise
    gyrNoiseStdDev = 0.02350    # standard deviation of gyroscope noise
    accBiasWalkStdDev = 1e-7    # standard deviation of accelerometer bias random walk
    gyrBiasWalkStdDev = 1e-6    # standard deviation of gyroscope bias random walk
    
    
    # lmfit helps us to minimise a multivariate function. create some parameters
    # (https://lmfit.github.io/lmfit-py/examples/example_brute.html)
    params = Parameters()
    # add the relevant parameters and their default values 
    # make sure the names are the same as in the .yaml file
    # set third argument to True if the parameter is variable
    # once optimised, can set all to false and use optimum values to function
        # as a normal comparison between estimate and ground truth
    params.add_many(
        ('max_cnt', 175, True),
        ('min_dist', 90, True),
        ('freq', 20, False),
        ('equalize', 1, False),
        ('acc_n', 0.02616, True),
        ('gyr_n', 0.02350, True),
        ('acc_w', 0.0000001, True),
        ('gyr_w', 0.000001, True))
    
    # configure the variable parameters with range and step size
    params['max_cnt'].set(min=50, max=500) # when using grid: include brute_step=100
    params['min_dist'].set(min=50, max=150)
    params['acc_n'].set(min=0.0005, max=0.5)
    params['gyr_n'].set(min=0.0005, max=0.5)
    params['acc_w'].set(min=0.0000001, max=0.001)
    params['gyr_w'].set(min=0.0000001, max=0.001)
    
    # obtain a minimum of the function for these parameters through brute-force
        # grid searching
        
    #fitter = Minimizer(averageScoreZ, params)
    #result = fitter.minimize(method='differential_evolution')
    
    result = minimize(fcn=averageScore, params=params, method='differential_evolution',args=None)
    
    print("Optimum Point: ")
    print(result.brute_x0)
    
    print("Score at Optimum Point: ")
    print(result.brute_fval)
"""    
def main():
    midairOnORB = importlib.import_module("midair_ORB-SLAM2")
    midairOnORB.main("/media/rory_haggart/ENDLESS_BLU/sceneVisibilityInSLAM/MidAir/MidAir/PLE_test/spring/trajectory_4000_color_left.bag")
    """
    ### VINS CONFIGURATION ###
        VINS-Mono has a range of configuration parameters for the particular
            dataset. This includes physical things like the camera calibration,
            and the sensor noises, as well as algorithm paramters like the 
            number of points to select and their minimum distance from one
            another.
        Some of these paramters are pretty fixed (e.g. camera calibration is
            a known thing), but others are less certain and require some level
            of tuning to bring the quality of tracking up
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
        
    """
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
    root = "/media/rory_haggart/ENDLESS_BLU/SLAM_datasets/MidAir"
    
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
    
    environment = "Kite_test" 
    condition = "sunny"
    trajectory = "0001"
    camera = "color_down"
    
    bagFilePath = root + "/MidAir/" + environment + "/" + condition +  "/trajectory_" + trajectory + "_" + camera + ".bag"
    
    # TODO: check if the bag file exists, if not, create it
    
    ### RUN TEST AND PRODUCE COMPARISON TO GROUND TRUTH ###
    # TODO: have a 'loop' parameter to execute this n times and take average
        # scores
    rollingSum = [0, 0, 0]
    numLoops = 3
    i = 0
    discontinuityCounter = 0
    while i < numLoops:
        SSE = midair_ORB-SLAM2.main(bagFilePath)
        
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
    
    
    
"""
In case of emergency, the .yaml file at time of writing (20/02/21) is below
performance of these paramters isn't *great* but it's an okay starting point

%YAML:1.0

#common parameters
imu_topic: "/imu0"
image_topic: "/cam0/image_raw"
output_path: "/home/rory_haggart/output"

#camera calibration 
# camera is ideal pinhole so no distortion. FAQ page https://midair.ulg.ac.be/faq.html tells us that the focal length is h / 2 = w / 2 = 512
model_type: PINHOLE
camera_name: camera
image_width: 1024
image_height: 1024
distortion_parameters:
   k1: 0
   k2: 0
   p1: 0
   p2: 0
projection_parameters:
   fx: 512
   fy: 512
   cx: 0
   cy: 0

# Extrinsic parameter between IMU and Camera.
estimate_extrinsic: 0   # 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                        # 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.
                        # 2  Don't know anything about extrinsic parameters. You don't need to give R,T. We will try to calibrate it. Do some rotation movement at beginning.                        
#If you choose 0 or 1, you should write down the following matrix.
#Rotation from camera frame to imu frame, imu^R_cam

# the nice thing about a synthesised data set is we don't have to worry about the extrinsic relation between frames. the down camera and imu are in the same frame!
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [-1, 0, 0,
          0, -1, 0, 
          0, 0, 1]

#TODO:: ROTATE CAMERA FROM BODY FRAME!! currently camera and body frame are at same rotation but need to match the website info???

#Translation from camera frame to imu frame, imu^T_cam
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0, 0, 0]

#feature traker paprameters
max_cnt: 200            # max feature number in feature tracking
min_dist: 90            # min distance between two features 
freq: 20                # frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
F_threshold: 1.0        # ransac threshold (pixel)
show_track: 1           # publish tracking image as topic
equalize: 1             # if image is too dark or light, trun on equalize to find enough features
fisheye: 0              # if using fisheye, trun on it. A circle mask will be loaded to remove edge noisy points

#optimization parameters
max_solver_time: 0.04  # max solver itration time (ms), to guarantee real time
max_num_iterations: 8   # max solver itrations, to guarantee real time
keyframe_parallax: 3.0 # keyframe selection threshold (pixel)

#imu parameters       The more accurate parameters you provide, the better performance
acc_n: 0.02616         # accelerometer measurement noise standard deviation. #0.05616
gyr_n: 0.02350         # gyroscope measurement noise standard deviation.     #0.04350
acc_w: 0.0000001       # accelerometer bias random work noise standard deviation.  #0.02
gyr_w: 0.000001        # gyroscope bias random work noise standard deviation.     #4.0e-5
g_norm: 9.8000     # gravity magnitude

#loop closure parameters
loop_closure: 1                    # start loop closure
load_previous_pose_graph: 0        # load and reuse previous pose graph; load from 'pose_graph_save_path'
fast_relocalization: 1             # useful in real-time and large project
pose_graph_save_path: "/home/rory_haggart/output/pose_graph/" # save and load path

#unsynchronization parameters
estimate_td: 0                      # online estimate time offset between camera and imu
td: 0.0                             # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)

#rolling shutter parameters
rolling_shutter: 0                  # 0: global shutter camera, 1: rolling shutter camera
rolling_shutter_tr: 0               # unit: s. rolling shutter read out time per frame (from data sheet). 

#visualization parameters
save_image: 0                   # save image in pose graph for visualization prupose; you can close this function by setting 0 
visualize_imu_forward: 0        # output imu forward propogation to achieve low latency and high frequence results
visualize_camera_size: 0.4      # size of camera marker in RVIZ
"""