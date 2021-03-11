import re
from os import listdir

root = "/home/rory_haggart/SLAM_datasets/Malaga/malaga_15/"

stereoImgPath = root + "malaga-urban-dataset-extract-15_rectified_1024x768_Images"
stereoFiles = sorted(listdir(stereoImgPath))

fileSter = open(root + "IMAGES_STEREO_TEMP.txt", "w")
for stereoFile in stereoFiles:
    fileSter.write(stereoFile + '\n')
fileSter.close()

file1 = open(root + "IMAGES_STEREO_TEMP.txt", "r")
malagaInfo = file1.readlines()
file1.close()

leftCamera = []

for img in malagaInfo:
    if "left" in img:
        leftCamera.append(img)
        
timeStepFormat = []

for img in leftCamera:
    timeStep = re.search("CAMERA1_(.*)_left", img)
    timeStepFormat.append(timeStep.group(1) + " /" + img)
    
file2 = open(root + "IMAGES_LEFT.txt", "w")
file2.writelines(timeStepFormat)
file2.close()