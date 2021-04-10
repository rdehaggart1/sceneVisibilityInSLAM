"""
    @author: rory_haggart
    
    BRIEF:  This script allows you to select one of the InteriorNet dataset extracts
            to convert into a ROS .bag file. 
            It will prompt you to select the extract and then use the 
            provided imu file and the images of this extract to 
            generate the .bag file.
    
    TODO:
        Work through the ground truth attitude -> local attitude process
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

def main():
    # enter function to ask for specific trajectory to bagify and return selection
    trajID = userInput()  
    
    # use the selection to find the appropriate sensor records file
    sensorRecordsPath = os.path.abspath(os.getcwd() + "/MidAir/" + environment + '/' + condition)
    sensorRecordsFile = sensorRecordsPath + "/sensor_records.hdf5"
    
    # if the sensor records file doesn't exist, or is not yet unzipped, exit
    if not os.path.exists(sensorRecordsFile):
        if os.path.exists(sensorRecordsPath + "/sensor_records.zip"):
            print("I did not find the file: " + sensorRecordsFile + "\n\n I did find the corresponding .zip file, however. Please uncompress this file and try again.")
        else:
            print("I did not find the file: " + sensorRecordsFile)
        sys.exit(0)
        
    # open sensor_records.hdf5
    f1 = h5py.File((sensorRecordsFile),'r+')
    
    # get imu readings plus the attitude of the vehicle
    accelerometer = f1['trajectory_' + trajectory]['imu']['accelerometer']
    gyroscope = f1['trajectory_' + trajectory]['imu']['gyroscope']
    groundTruthAttitude = f1['trajectory_' + trajectory]['groundtruth']['attitude']
    
    # get the accelerometer data from the sensor records (m/s^2)
    accelerometerData = list(accelerometer) 
    # get the gyroscope data from the sensor records (rad/s)
    gyroscopeData = list(gyroscope) 
    # list the relative paths of the images for the selected camera
    imagePaths = list(f1['trajectory_' + trajectory]['camera_data'][camera]) 
    
    # exit if the selected trajectory images haven't been unzipped yet
    if (not any('.JPEG' in a for a in os.listdir(sensorRecordsPath + "/" + camera + "/trajectory_" + trajectory))) and any('.zip' in a for a in os.listdir(sensorRecordsPath + "/" + camera + "/trajectory_" + trajectory)):
        print("The images for this particular trajectory have not yet been unzipped.\nPlease unzip and try again.")
        sys.exit(0)
    
    # .bag file name is of the format trajectoryNumber_camera.bag and is located in environment/condition
    bagFilePath = sensorRecordsPath + "/trajectory_" + trajectory + "_" + camera + ".bag"
    
    # just check the user is okay if overwriting a bag that already exists
    if(os.path.isfile(bagFilePath)):
        answer = input("The .bag file {} already exists. Would you like to overwrite? (y/n)\n".format(bagFilePath))
        if(answer=='n' or answer=='N'):
            sys.exit(0)
    
    # open our .bag file to write to
    bag = rosbag.Bag(bagFilePath, "w")
    
    # initialise sequence number
    seq = 0
    # define camera frame rate
    cameraRate = 25;
    # an arbitrary starting time (seconds since epoch). mostly protecting against the invalid '0' time
    initialTime = 100000;
    
    # image dimensions
    imHeight = 1024
    imWidth = 1024
    
    for line in imagePaths:
        absImg = sensorRecordsPath + "/" + line # get the absolute file path
        
        # ---------- IMAGE RECORD ---------- #
        # if the image exists and isn't a dud
        if (os.path.isfile(absImg)) and (os.stat(absImg).st_size != 0):
    
            print("Adding %s" % absImg[absImg.rindex('/') + 1:])
    
            # read the image
            image = cv2.imread(absImg)
            
            # convert from cv2 to ROS by creating an Image(). This auto allocates
                # the image data to the .data field so headers/config can be
                # added below
            bridge = CvBridge()
            #img_msg.data = image
            img_msg = bridge.cv2_to_imgmsg(image, "passthrough")
            
            # img_msg format: 
            # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
            
            # get the timestamp of this frame
            timestamp = seq * 1/cameraRate + initialTime
            # the seconds part of the timestamp is just the whole number
            timestampSec = math.floor(timestamp)
            # the nanoseconds part of the timestamp is then the decimal part
            timestampnSec = int(round(timestamp - timestampSec, 3) * pow(10, 9))
            
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
        else:
            print("Could not find %s" % absImg[absImg.rindex('/') + 1:])
            
    # reset sequence ID for IMU recording
    seq = 0
    
    # the update rate of the IMU is 100Hz
    imuRate = 100
    
    print("Adding IMU Data")
    
    for accelLine, gyroLine, attitudeLine in zip(accelerometerData, gyroscopeData, groundTruthAttitude):
        
        # get the timestamp of this frame
        timestamp = seq * 1/imuRate + initialTime
        # the seconds part of the timestamp is just the whole number
        timestampSec = math.floor(timestamp)
        # the nanoseconds part of the timestamp is then the decimal part
        timestampnSec = int(round(timestamp - timestampSec, 3) * pow(10, 9))
        
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
        
        # attitude (quaternion)
        imu_msg.orientation.w = attitudeLine[0]
        imu_msg.orientation.x = attitudeLine[1]
        imu_msg.orientation.y = attitudeLine[2]
        imu_msg.orientation.z = attitudeLine[3]
        
        # write the imu_msg to the bag file
        bag.write("imu0", imu_msg, imu_msg.header.stamp) 
    
        # increment the sequence counter
        seq = seq + 1
    
    # close the .bag file
    bag.close()
    # close the .hdf5 file
    f1.close()


# global flags for selection prompts
installFlag = ""
notInstallFlag = "(NOT INSTALLED)"

# prompt the user through the process of selecting the trajectory to bagify
def userInput():
    # get the path to the dataset folder
    dataPath = os.getcwd() + "/InteriorNet"
    
    # get all (unzipped) folders available to the user
    trajectoryFolders = next(os.walk(os.path.join(dataPath,'.')))[1]
    
    print("Please select the trajectory to bagify\n")
    for i in range(len(trajectoryFolders)):
        print("{}. {}".format(i+1, trajectoryFolders[i]))
    
    # get the selected index
    trajIndex = int(input(""))
    
    if trajIndex not in range(1,len(trajectoryFolders)+1):
        sys.exit("Out of range value entered")
    
    # then get the name of the trajectory based on the selection
    trajectoryName = trajectoryFolders[trajIndex - 1]
    
    ## TODO: get the lighting and traj number e.g. random, 3
        
    return trajectoryName

if __name__ == "__main__":
    main()