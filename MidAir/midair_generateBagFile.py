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
    [environment, condition, trajectory, camera] = userInput()  
    
    # use the selection to find the appropriate sensor records file
    sensorRecordsPath = os.path.abspath(os.getcwd() + "/MidAir/" + environment + '/' + condition)
    sensorRecordsFile = sensorRecordsPath + "/sensor_records.hdf5"
    
    # if the sensor records file doesn't exist, exit
    if not os.path.exists(sensorRecordsFile):
        if os.path.exists(sensorRecordsPath + "/sensor_records.zip"):
            print("I did not find the file: " + sensorRecordsFile + "\n\n I did find the corresponding .zip file, however. Please uncompress this file and try again.")
        else:
            print("I did not find the file: " + sensorRecordsFile)
        sys.exit(0)
        
    # open sensor_records.hdf5
    f1 = h5py.File((sensorRecordsPath + '/sensor_records.hdf5'),'r+')
    
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
        
        # currently, no data on local attitude is available, so as per message
            # standard, set orientation covariance to -1
            # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
        # we do have ground truth attitude so should be able to extract local
            # angles, but not completed yet
        imu_msg.orientation_covariance = [-1 for i in imu_msg.orientation_covariance]
        
        # attitude (quaternion)
        #imu_msg.orientation.x = attitudeLine[1]
        #imu_msg.orientation.y = attitudeLine[2]
        #imu_msg.orientation.z = attitudeLine[3]
        #imu_msg.orientation.w = attitudeLine[0]
        
        # write the imu_msg to the bag file
        bag.write("imu0", imu_msg, imu_msg.header.stamp) 
    
        # increment the sequence counter
        seq = seq + 1
    
    # close the .bag file
    bag.close()
    # close the .hdf5 file
    f1.close()

# prompt the user through the process of selecting the trajectory to bagify
def userInput():
    dataPath = os.getcwd() + "/MidAir"
    
    installFlag = ""
    notInstallFlag = "(NOT INSTALLED)"
    
    kiteTestFlag = installFlag if os.path.isdir(dataPath + "/Kite_test") else notInstallFlag
    kiteTrainFlag = installFlag if os.path.isdir(dataPath + "/Kite_training") else notInstallFlag
    pleTestFlag = installFlag if os.path.isdir(dataPath + "/PLE_test") else notInstallFlag
    pleTrainFlag = installFlag if os.path.isdir(dataPath + "/PLE_training") else notInstallFlag
    voTestFlag = installFlag if os.path.isdir(dataPath + "/VO_test") else notInstallFlag
    
    answer = int(input("""Please enter the environment you are testing in:\n
    1. Kite_test {}
    2. Kite_training {}               
    3. PLE_test {}
    4. PLE_training {}
    5. VO_test {}\n\n""".format(kiteTestFlag, kiteTrainFlag,pleTestFlag,pleTrainFlag,voTestFlag)))
    
    if (answer==1):
        if kiteTestFlag == notInstallFlag:
            print("Environment not installed")
            sys.exit(0)
        else:
            environment="Kite_test"
    elif(answer==2):
        if kiteTrainFlag == notInstallFlag:
            print("Environment not installed")
            sys.exit(0)
        else:
            environment="Kite_training"
    elif(answer==3):
        if pleTestFlag == notInstallFlag:
            print("Environment not installed")
            sys.exit(0)
        else:
            environment="PLE_test"
    elif(answer==4):
        if pleTrainFlag == notInstallFlag:
            print("Environment not installed")
            sys.exit(0)
        else:
            environment="PLE_training"
    elif(answer==5):
        if voTestFlag == notInstallFlag:
            print("Environment not installed")
            sys.exit(0)
        else:
            environment="VO_test"
    else:
        sys.exit("You entered an out-of-range value")
        
    if "Kite" in environment:
        trajRange = 4 if("test" in environment) else 29
        
        cloudyFlag = installFlag if os.path.isdir(dataPath + "/" + environment + "/" + "cloudy") else notInstallFlag
        foggyFlag = installFlag if os.path.isdir(dataPath + "/" + environment + "/" + "foggy") else notInstallFlag
        sunnyFlag = installFlag if os.path.isdir(dataPath + "/" + environment + "/" + "sunny") else notInstallFlag
        sunsetFlag = installFlag if os.path.isdir(dataPath + "/" + environment + "/" + "sunset") else notInstallFlag
        
        answer = int(input("""Please enter the condition you are testing in:\n
    1. cloudy {}
    2. foggy {}           
    3. sunny {}
    4. sunset {}\n\n""".format(cloudyFlag,foggyFlag,sunnyFlag,sunsetFlag)))
        if (answer==1):
            if cloudyFlag == notInstallFlag:
                print("Condition not installed")
                sys.exit(0)
            else:
                condition="cloudy"
                
                trajSearchPath = dataPath + "/" + environment + "/" + condition
                trajNo = trajPrinter(trajSearchPath, trajRange)
                if(trajNo > trajRange or trajNo < 0):
                    sys.exit("You entered an out-of-range value")
                trajectory = "3" + str(trajNo).zfill(3)
        elif(answer==2):
            if foggyFlag == notInstallFlag:
                print("Condition not installed")
                sys.exit(0)
            else:
                condition="foggy"

                trajSearchPath = dataPath + "/" + environment + "/" + condition
                trajNo = trajPrinter(trajSearchPath, trajRange)
                if(trajNo > trajRange or trajNo < 0):
                    sys.exit("You entered an out-of-range value")
                trajectory = "2" + str(trajNo).zfill(3)
                
        elif(answer==3):
            if sunnyFlag == notInstallFlag:
                print("Condition not installed")
                sys.exit(0)
            else:
                condition="sunny"
                
                trajSearchPath = dataPath + "/" + environment + "/" + condition
                trajNo = trajPrinter(trajSearchPath, trajRange)
                if(trajNo > trajRange or trajNo < 0):
                    sys.exit("You entered an out-of-range value")
                trajectory = "0" + str(trajNo).zfill(3)
        elif(answer==4):
            if sunsetFlag == notInstallFlag:
                print("Condition not installed")
                sys.exit(0)
            else:
                condition="sunset"
                
                trajSearchPath = dataPath + "/" + environment + "/" + condition
                trajNo = trajPrinter(trajSearchPath, trajRange)
                if(trajNo > trajRange or trajNo < 0):
                    sys.exit("You entered an out-of-range value")
                trajectory = "1" + str(trajNo).zfill(3)
        else:
            sys.exit(0)
            
    elif "PLE" in environment:
        trajRange = 5 if("test" in environment) else 23
        
        answer = int(input("""Please enter the condition you are testing in:\n
    1. fall
    2. spring               
    3. winter\n\n"""))
        if (answer==1):
            condition="fall"
            
            trajSearchPath = dataPath + "/" + environment + "/" + condition
            trajNo = trajPrinter(trajSearchPath, trajRange)
            if(trajNo > trajRange or trajNo < 0):
                sys.exit("You entered an out-of-range value")
        elif(answer==2):
            condition="spring"
            
            trajSearchPath = dataPath + "/" + environment + "/" + condition
            trajNo = trajPrinter(trajSearchPath, trajRange)
            if(trajNo > trajRange or trajNo < 0):
                sys.exit("You entered an out-of-range value")
        elif(answer==3):
            condition="winter"
            
            trajSearchPath = dataPath + "/" + environment + "/" + condition
            trajNo = trajPrinter(trajSearchPath, trajRange)
            if(trajNo > trajRange or trajNo < 0):
                sys.exit("You entered an out-of-range value")
        else:
            sys.exit(0)    
            
        trajectory = "4" + str(trajNo).zfill(3)
                
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
            
            trajSearchPath = dataPath + "/" + environment + "/" + condition
            trajNo = trajPrinter(trajSearchPath, trajRange)
            if(trajNo > trajRange or trajNo < 0):
                sys.exit("You entered an out-of-range value")
            trajectory = "1" + str(trajNo).zfill(3)
        elif(answer==2):
            condition="sunny"
            
            trajSearchPath = dataPath + "/" + environment + "/" + condition
            trajNo = trajPrinter(trajSearchPath, trajRange)
            if(trajNo > trajRange or trajNo < 0):
                sys.exit("You entered an out-of-range value")
            trajectory = "0" + str(trajNo).zfill(3)
        elif(answer==3):
            condition="sunset"
            
            trajSearchPath = dataPath + "/" + environment + "/" + condition
            trajNo = trajPrinter(trajSearchPath, trajRange)
            if(trajNo > trajRange or trajNo < 0):
                sys.exit("You entered an out-of-range value")
            trajectory = "2" + str(trajNo).zfill(3)
        else:
            sys.exit("You entered an invalid value")
    
    colorLeftFlag =  installFlag if os.path.isdir(dataPath + "/" + environment + "/" + condition + "/color_left") else notInstallFlag   
    colorRightFlag =  installFlag if os.path.isdir(dataPath + "/" + environment + "/" + condition + "/color_right") else notInstallFlag 
    colorDownFlag =  installFlag if os.path.isdir(dataPath + "/" + environment + "/" + condition + "/color_down") else notInstallFlag 
    
    answer = int(input("""Please enter the camera you are testing with:\n
    1. color_left {}
    2. color_right {}             
    3. color_down {} \n\n""".format(colorLeftFlag,colorRightFlag,colorDownFlag)))
    
    if (answer==1):
        if colorLeftFlag == notInstallFlag:
            print("Camera not installed")
            sys.exit(0)
        else:
            camera="color_left"
    elif(answer==2):
        if colorRightFlag == notInstallFlag:
            print("Camera not installed")
            sys.exit(0)
        else:
            camera="color_right"
    elif(answer==3):
        if colorDownFlag == notInstallFlag:
            print("Camera not installed")
            sys.exit(0)
        else:
            camera="color_down"
    else:
        sys.exit("You entered an invalid value") 
        
    return [environment, condition, trajectory, camera]
    
# print trajectory numbers with notice of whether or not they are installed
def trajPrinter(trajSearchPath, trajRange):
    trajList = list(Path(trajSearchPath).rglob("[trajectory]*"))
    trajList = [str(a) for a in trajList if ("trajectory" in str(a) and ".bag" not in str(a))]
    trajList = [int(a[-2:]) for a in trajList]
    
    print("Please select the trajectory to test:\n")
    for i in range(trajRange + 1):
        trajFlag = "" if i in trajList else "(NOT INSTALLED)"
        print("    {}. {}".format(i, trajFlag))
        
    trajNo = int(input("\n"))
    
    if trajNo not in trajList:
        print("Trajectory not installed")
        sys.exit(0)
    
    return(trajNo)

if __name__ == "__main__":
    main()