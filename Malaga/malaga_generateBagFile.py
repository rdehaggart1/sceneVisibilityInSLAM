"""
    @author: rory_haggart
    BRIEF: create a .bag file from a selected extract from the malaga dataset
    
    First extract only the left images for monocular use. stereo also easily
        achievable from this framework but for this use case monocular allows
        for smaller file size and less creation time for bigger extracts
    Then get all of the IMU data and timestamps and package into .bag
"""

# for .bag writing 
import rosbag
# to store the IMU/INS measurements as a ROS message
from sensor_msgs.msg import Imu
# for image interpretation and manipulation
import cv2
# to convert from a cv2 image to a ROS compatible image data
from cv_bridge import CvBridge
# to read .hdf5 sensor records files
import h5py    
import os
import math
import matplotlib.pyplot as plt
import numpy as np
import re
import shlex

extractNumber = 15; # the number of the malaga dataset extract

# save the full path of the extract folder
extractPath = os.path.abspath(os.getcwd() + "/Malaga/malaga_{}/".format(extractNumber))

# get the folder of the stereo images for this extract
stereoImgPath = extractPath + "/Images".format(extractNumber)
# get all of the images in the stereo folder
stereoPaths = sorted(os.listdir(stereoImgPath))

# lists for the file path and the timestamp of each left camera image
leftCameraPaths = []
leftCameraTimestamps = []

# for each of the stero files
for img in stereoPaths:
    if "left" in img:
        # store all files that are from the left camera
        leftCameraPaths.append(stereoImgPath + "/" + img)
        
        # and separately store all of their timestamps
        timestamp = re.search("CAMERA1_(.*)_left", img)
        leftCameraTimestamps.append(float(timestamp.group(1)))
        
# image dimensions
imWidth = 1024
imHeight = 768

# read the collected imu data into a list
imuDataPath = extractPath + "/malaga-urban-dataset-extract-{}_all-sensors_IMU.txt".format(extractNumber)
with open(imuDataPath) as f1:
    imuDataFile = f1.readlines()
    
# split the list items by spaces
imuData = []
for line in imuDataFile:
    imuData.append(shlex.split(line))

# get particular readings of interest
imuDataTimestamps = [float(row[0]) for row in imuData[1:]]
imuDataAccX = [float(row[1]) for row in imuData[1:]]
imuDataAccY = [float(row[2]) for row in imuData[1:]]
imuDataAccZ = [float(row[3]) for row in imuData[1:]]
imuDataGyrX = [float(row[6]) for row in imuData[1:]]    # x is forward (roll)
imuDataGyrY = [float(row[5]) for row in imuData[1:]]    # y is left (pitch)
imuDataGyrZ = [float(row[4]) for row in imuData[1:]]    # z is upward (yaw)

# concatenate the x,y,z readings into individual rows
accelerometerData = [[imuDataAccX[i], imuDataAccY[i], imuDataAccZ[i]] for i in range(len(imuDataTimestamps))]
gyroscopeData = [[imuDataGyrX[i], imuDataGyrY[i], imuDataGyrZ[i]] for i in range(len(imuDataTimestamps))]

seq = 0 # each image should be given an index in the sequence    

# open our .bag file to write to
bag = rosbag.Bag(extractPath + "/malaga_{}".format(extractNumber) + ".bag", "w")

for imagePath, imageTimestamp in zip(leftCameraPaths, leftCameraTimestamps):
    # ---------- IMAGE RECORD ---------- #
    # if the image exists and isn't a dud
    if (os.path.isfile(imagePath)) and (os.stat(imagePath).st_size != 0):

        print("Adding {}: ".format(seq) + "{}".format(imagePath[imagePath.rindex('/') + 1:]))

        # read the image
        image = cv2.imread(imagePath)
        
        # convert from cv2 to ROS by creating an Image(). This auto allocates
            # the image data to the .data field so headers/config can be
            # added below
        bridge = CvBridge()
        #img_msg.data = image
        img_msg = bridge.cv2_to_imgmsg(image, "passthrough")
        
        # img_msg format: 
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        
        # get the timestamp of this frame
        timestamp = imageTimestamp
        # the seconds part of the timestamp is just the whole number
        timestampSec = math.floor(timestamp)
        # the nanoseconds part of the timestamp is then the decimal part
        timestampnSec = int((timestamp - timestampSec) * pow(10, 9))
        
        # set image timestamp
        img_msg.header.stamp.secs = timestampSec
        img_msg.header.stamp.nsecs = timestampnSec
        # number in sequence
        img_msg.header.seq = seq
        # frame source
        img_msg.header.frame_id = "cam0"
        # dimensions
        img_msg.width = imWidth
        img_msg.height = imHeight
        img_msg.step = imWidth * 3  # (image width in bytes)
        # encoding
        img_msg.encoding = "bgr8"
        
        # write image to the bag file under the 'cam0/image_raw' topic
        bag.write("cam0/image_raw", img_msg, img_msg.header.stamp)
        
        # increment seq num
        seq = seq + 1

# reset sequence ID for IMU recording
seq = 0

for measurementTimestamp, accelLine, gyroLine in zip(imuDataTimestamps, accelerometerData, gyroscopeData):
    # get the timestamp of this frame
    timestamp = measurementTimestamp
    # the seconds part of the timestamp is just the whole number
    timestampSec = math.floor(timestamp)
    # the nanoseconds part of the timestamp is then the decimal part
    timestampnSec = int((timestamp - timestampSec) * pow(10, 9))
    
    # create an imu_msg for our inertial data
    imu_msg = Imu()
    
    # imu_msg format: 
    # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
    
    # ---------- IMU RECORD ---------- #
    # header info
    imu_msg.header.frame_id = "imu0"
    imu_msg.header.seq = seq
    imu_msg.header.stamp.secs = timestampSec
    # microseconds to nanoseconds again
    imu_msg.header.stamp.nsecs = timestampnSec
    
    # linear accelerations (m/s^2)
    imu_msg.linear_acceleration.x = accelLine[0]
    imu_msg.linear_acceleration.y = accelLine[1]
    imu_msg.linear_acceleration.z = accelLine[2]
    
    # angular rates (rad/s)
    imu_msg.angular_velocity.x = gyroLine[0]
    imu_msg.angular_velocity.y = gyroLine[1]
    imu_msg.angular_velocity.z = gyroLine[2]
    
    # TODO: attitude
    
    # get the roll/pitch/yaw values (radians)
    #roll = float(splitLine[17])
    #pitch = float(splitLine[18])
    #yaw = float(splitLine[19])
 
    # generate quaternions from euler angles & assign
    #qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    #qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    #qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    #qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    #imu_msg.orientation.x = attitudeLine[1]
    #imu_msg.orientation.y = attitudeLine[2]
    #imu_msg.orientation.z = attitudeLine[3]
    #imu_msg.orientation.w = attitudeLine[0]
    
    imu_msg.orientation_covariance = [-1 for i in imu_msg.orientation_covariance]
    
    # write the imu_msg to the bag file
    bag.write("imu0", imu_msg, imu_msg.header.stamp) 

    # increment the sequence counter
    seq = seq + 1

# close the .bag file
bag.close()