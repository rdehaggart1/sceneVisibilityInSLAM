"""
    @author: rory_haggart
    
    BRIEF:  This script allows you to select one of the InteriorNet dataset extracts
            to convert into a ROS .bag file. 
            It will prompt you to select the extract and then use the 
            provided imu file and the images of this extract to 
            generate the .bag file.
            
            The bag file will contain the RGB camera images from cam0 under 
            the /camera/image_raw rostopic, and the imu data (accelerometer & 
            gyroscope) under the /imu rostopic
"""

# for bagification
import rosbag
# to store the IMU/INS measurements as a ROS message
from sensor_msgs.msg import Imu
# for image interpretation and manipulation
import cv2
# to convert from a cv2 image to a ROS compatible image data
from cv_bridge import CvBridge
# to read .hdf5 sensor records files
import h5py
# for file path interpretation and representation    
import os
# for timestamp operations
import math
# for exit control
import sys
# for some file representation stuff
from pathlib import Path

import pandas as pd

def main():
    # enter function to ask for specific trajectory to bagify and return selection
    trajID, extID = userInput()  
    
    extNum = extID[-1]
    
    # use the selection to find the appropriate sensor records file
    imuDataFile = os.path.abspath(os.getcwd() + "/InteriorNet/" + trajID + "/velocity_angular_" + "{}_{}".format(extNum, extNum) + "/imu0/data.csv")
    
    # if the sensor records file doesn't exist, or is not yet unzipped, exit
    if not os.path.exists(imuDataFile):
        print("I did not find the file: " + imuDataFile)
        sys.exit(0)
    
    imuData = pd.read_csv(imuDataFile)
    
    imuTimestamps = [ts * pow(10,-9) for ts in list(imuData['#timestamp [ns]'])]
    accelerometer = [[x, y, z] for x, y, z in zip(imuData['a_RS_S_x [m s^-2]'],
                                                  imuData['a_RS_S_y [m s^-2]'], 
                                                  imuData['a_RS_S_z [m s^-2]'])]
    gyroscope = [[x, y, z] for x, y, z in zip(imuData['w_RS_S_x [rad s^-1]'],
                                              imuData['w_RS_S_y [rad s^-1]'], 
                                              imuData['w_RS_S_z [rad s^-1]'])]
    
    # list the relative paths of the images for the selected camera
    imgDataFile = os.getcwd() + "/InteriorNet/" + trajID + "/" + extID + "/cam0/data.csv"
    
    imgData = pd.read_csv(imgDataFile)
    
    imgTimestamps = [ts * pow(10,-9) for ts in list(imgData['#timestamp [ns]'])]
    imgPaths = [(os.getcwd() + "/InteriorNet/" + trajID + "/" + extID + "/cam0/data/" + imgName) for imgName in list(imgData['filename'])]
    
    # .bag file name is of the format trajectoryNumber_camera.bag and is located in environment/condition
    bagFilePath = os.getcwd() + "/InteriorNet/" + trajID + "/" + extID + ".bag"
    
    # just check the user is okay if overwriting a bag that already exists
    if(os.path.isfile(bagFilePath)):
        answer = input("The .bag file {} already exists. Would you like to overwrite? (y/n)\n".format(bagFilePath))
        if(answer=='n' or answer=='N'):
            sys.exit(0)
    
    # open our .bag file to write to
    bag = rosbag.Bag(bagFilePath, "w")
    
    # initialise sequence number
    seq = 0
    
    # image dimensions
    imHeight = 480
    imWidth = 640
    
    for img, ts in zip(imgPaths, imgTimestamps):
        # ---------- IMAGE RECORD ---------- #
        # if the image exists and isn't a dud
        if (os.path.isfile(img)) and (os.stat(img).st_size != 0):
    
            print("Adding Image {}/{}".format(seq, len(imgPaths)))
    
            # read the image
            image = cv2.imread(img)
            
            # convert from cv2 to ROS by creating an Image(). This auto allocates
                # the image data to the .data field so headers/config can be
                # added below
            bridge = CvBridge()
            #img_msg.data = image
            img_msg = bridge.cv2_to_imgmsg(image, "passthrough")
            
            # img_msg format: 
            # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
            
            # the seconds part of the timestamp is just the whole number
            timestampSec = math.floor(ts)
            # the nanoseconds part of the timestamp is then the decimal part
            timestampnSec = int(round(ts - timestampSec, 8) * pow(10, 9))
            
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
            bag.write("camera/image_raw", img_msg, img_msg.header.stamp)
            
            # increment seq num
            seq = seq + 1
        else:
            print("Could not find %s" % img[img.rindex('/') + 1:])
            
    # reset sequence ID for IMU recording
    seq = 0
    
    print("Adding IMU Data")
    
    for accel, gyro, ts in zip(accelerometer, gyroscope, imuTimestamps):
        # the seconds part of the timestamp is just the whole number
        timestampSec = math.floor(ts)
        # the nanoseconds part of the timestamp is then the decimal part
        timestampnSec = int(round(ts - timestampSec, 8) * pow(10, 9))
        
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
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        
        # angular rates (rad/s)
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        
        # currently, no IMU data on attitude is available, so as per message
            # standard, set orientation covariance to -1
            # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
        # we do have ground truth attitude so could use this if we want to
        imu_msg.orientation_covariance = [-1 for i in imu_msg.orientation_covariance]
        
        # write the imu_msg to the bag file
        bag.write("imu", imu_msg, imu_msg.header.stamp) 
    
        # increment the sequence counter
        seq = seq + 1
    
    # close the .bag file
    bag.close()


# prompt the user through the process of selecting the trajectory to bagify
def userInput():
    # get the path to the dataset folder
    dataPath = os.getcwd() + "/InteriorNet"
    
    # get all (unzipped) folders available to the user
    trajectoryFolders = next(os.walk(os.path.join(dataPath,'.')))[1]
    
    if len(trajectoryFolders)==0:
        print("No trajectories available. Make sure you have downloaded and unzipped some InteriorNet files to {}".format(dataPath))
        sys.exit(0)
    
    print("Please select a trajectory\n")
    for i in range(len(trajectoryFolders)):
        print("{}. {}".format(i+1, trajectoryFolders[i]))
    
    # get the selected index
    trajIndex = int(input(""))
    
    if trajIndex not in range(1,len(trajectoryFolders)+1):
        sys.exit("Out of range value entered")
    
    # then get the name of the trajectory based on the selection
    trajectoryName = trajectoryFolders[trajIndex - 1]
    
    # get all extracts for this trajectory
    extractFolders = next(os.walk(os.path.join(dataPath + "/" + trajectoryName,'.')))[1]
    # remove the ground truth folders
    extractFolders = [ext for ext in extractFolders if 'velocity_angular' not in ext]
    
    print("Please select the extract to bagify\n")
    for i in range(len(extractFolders)):
        print("{}. {}".format(i+1, extractFolders[i]))
    
    # get the selected index
    extIndex = int(input(""))
    
    if extIndex not in range(1,len(extractFolders)+1):
        sys.exit("Out of range value entered")
    
    # then get the name of the trajectory based on the selection
    extractName = extractFolders[extIndex - 1]
    
    return trajectoryName, extractName

if __name__ == "__main__":
    main()