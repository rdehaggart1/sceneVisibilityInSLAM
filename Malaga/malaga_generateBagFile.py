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

# close the .bag file
bag.close()