#timeStep = 0.0
#for i in (num for num in range(326)):
#    numSz = len(str(i))
#    numZeros = 6 - numSz
#    zeroStr = ""
#    for j in range(numZeros):
#        zeroStr = zeroStr + "0"
#    #print('{:.5f} {}{}.JPEG'.format(timeStep, zeroStr, i))
#    print('{:.5f} {:.5f}.jpg'.format(timeStep, timeStep))
#    timeStep = timeStep + 1/25

#import os
#fnames = sorted(os.listdir('.'))
#fnames = fnames[0:-2]
#timeStep = 0.0
#for filename in fnames:
#    os.rename(filename, '{:.5f}.jpg'.format(timeStep))
#    timeStep = timeStep + 1/25


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
import sys

answer = int(input("""Please enter the environment you are testing in:\n
    1. Kite_test
    2. Kite_training               
    3. PLE_test
    4. PLE_training
    5. VO_test\n\n"""))

if (answer==1):
    environment="Kite_test"
elif(answer==2):
    environment="Kite_training"
elif(answer==3):
    environment="PLE_test"
elif(answer==4):
    environment="PLE_training"
elif(answer==5):
    environment="VO_test"
else:
    sys.exit("You entered an out-of-range value")
    
if "Kite" in environment:
    trajRange = 4 if("test" in environment) else 29
    trajNo = int(input("""Please enter trajectory number (0-{}):\n\n""".format(trajRange)))
    
    if(trajNo > trajRange or trajNo < 0):
        sys.exit("You entered an out-of-range value")
    
    answer = int(input("""Please enter the condition you are testing in:\n
    1. cloudy
    2. foggy               
    3. sunny
    4. sunset\n\n"""))
    if (answer==1):
        condition="cloudy"
        trajectory = "3" + str(trajNo).zfill(3)
    elif(answer==2):
        condition="foggy"
        trajectory = "2" + str(trajNo).zfill(3)
    elif(answer==3):
        condition="sunny"
        trajectory = "0" + str(trajNo).zfill(3)
    elif(answer==4):
        condition="sunset"
        trajectory = "1" + str(trajNo).zfill(3)
    else:
        sys.exit(0)
elif "PLE" in environment:
    trajRange = 5 if("test" in environment) else 23
    trajNo = int(input("""Please enter trajectory number (0-{}):\n\n""".format(trajRange)))
    
    if(trajNo > trajRange or trajNo < 0):
        sys.exit("You entered an out-of-range value")
    
    trajectory = "4" + str(trajNo).zfill(3)
    
    answer = int(input("""Please enter the condition you are testing in:\n
    1. fall
    2. spring               
    3. winter\n\n"""))
    if (answer==1):
        condition="fall"
    elif(answer==2):
        condition="spring"
    elif(answer==3):
        condition="winter"
    else:
        sys.exit(0)    
elif(environment=="VO_test"):
    trajRange = 2
    trajNo = int(input("""Please enter trajectory number (0-{}):\n\n""".format(trajRange)))
    
    if(trajNo > trajRange or trajNo < 0):
        sys.exit("You entered an out-of-range value")
    
    answer = int(input("""Please enter the condition you are testing in:\n
    1. foggy               
    2. sunny
    3. sunset\n\n"""))
    if(answer==1):
        condition="foggy"
        trajectory = "1" + str(trajNo).zfill(3)
    elif(answer==2):
        condition="sunny"
        trajectory = "0" + str(trajNo).zfill(3)
    elif(answer==3):
        condition="sunset"
        trajectory = "2" + str(trajNo).zfill(3)
    else:
        sys.exit("You entered an invalid value")
    
answer = int(input("""Please enter the camera you are testing with:\n
    1. color_left
    2. color_right               
    3. color_down\n\n"""))

if (answer==1):
    camera="color_left"
elif(answer==2):
    camera="color_right"
elif(answer==3):
    camera="color_down"
else:
    sys.exit("You entered an invalid value")   

# define the path to the folder containing our sensor records
sensorRecordsPath = os.path.abspath(os.getcwd() + "/MidAir/" + environment + '/' + condition)
sensorRecordsFile = sensorRecordsPath + "/sensor_records.hdf5"

assert os.path.exists(sensorRecordsFile), "\nI did not find the file: " + sensorRecordsFile + """\n\nThis could mean that you haven't downloaded this segment, or that the sensor_records file hasn't been unzipped yet"""

f1 = h5py.File((sensorRecordsPath + '/sensor_records.hdf5'),'r+')   # open sensor_records.hdf5

# list(f1['trajectory_0001'].keys()) to see keys in file

accelerometer = f1['trajectory_' + trajectory]['imu']['accelerometer']
gyroscope = f1['trajectory_' + trajectory]['imu']['gyroscope']
groundTruthAttitude = f1['trajectory_' + trajectory]['groundtruth']['attitude']

# get the accelerometer data from the sensor records (m/s^2)
accelerometerData = list(accelerometer) 
# get the gyroscope data from the sensor records (rad/s)
gyroscopeData = list(gyroscope) 
# list the relative paths of the images for the selected camera
imagePaths = list(f1['trajectory_' + trajectory]['camera_data'][camera]) 

#accelerationGroundTruth = f1['trajectory_' + trajectory]['groundtruth']['acceleration']
#timeSeries = np.arange(0, len(accelerationGroundTruth)/100, 1/100).tolist()
#xAccelerationTrue = [row[0] for row in accelerationGroundTruth]
#xAcceleration = [row[0] for row in accelerometerData]
#plt.plot(timeSeries, xAccelerationTrue, color="green")
#plt.plot(timeSeries, list(xAcceleration), color="red")
#plt.show()

# note the provided estimations of the initial sensor biases
accelerometerInitBiasEst = accelerometer.attrs['init_bias_est'][0]
gyroscopeInitBiasEst = gyroscope.attrs['init_bias_est'][0]

bagFilePath = sensorRecordsPath + "/trajectory_" + trajectory + "_" + camera + ".bag"

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
     
    # get the roll/pitch/yaw values (radians)
    #roll = float(splitLine[17])
    #pitch = float(splitLine[18])
    #yaw = float(splitLine[19])
 
    # generate quaternions from euler angles & assign
    #qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    #qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    #qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    #qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    imu_msg.orientation.x = attitudeLine[1]
    imu_msg.orientation.y = attitudeLine[2]
    imu_msg.orientation.z = attitudeLine[3]
    imu_msg.orientation.w = attitudeLine[0]
    
    #imu_msg.orientation_covariance = [-1 for i in imu_msg.orientation_covariance]
    
    # write the imu_msg to the bag file
    bag.write("imu0", imu_msg, imu_msg.header.stamp) 

    # increment the sequence counter
    seq = seq + 1

# close the .bag file
bag.close()
# close the .hdf5 file
f1.close()