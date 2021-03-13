# Online Scene Visibility Estimation as a Complement to SLAM in UAVs
As part of the final year research project with the University of Sheffield, with the title above, this repository contains supplementary code
to format data and present results. There are two folders contained in this root:
- Malaga
- MidAir


# 1. Prerequisites 
## ROS
## ORB-SLAM2
## VINS-Mono
## Python
## Packages

# 2. Installation
## Cloning
To clone this repository onto your machine, open a terminal in the desired cloning location and run the command:

`git clone https://github.com/rdehaggart1/RoryHaggart_SceneVisibilityInSLAM.git`

## Getting Datasets
Once you have the repository on your machine, you will need some data from the Malaga and MidAir datasets to run the code on
### Malaga

### MidAir
To download some data for the MidAir dataset:
1. Go to https://midair.ulg.ac.be/download.html
2. Select all the image types that you are interested in (this code has been tested only on the RGB images), and select the segments from the different environments that you would like to test. Note: the 'Test data for benchmarks' section at the bottom may be a useful starting point as these are short segments that are useful for establishing a good baseline
3. Click get download links and enter your email address. You will recieve a file containing the links to the data you have selected - download this file into the repository folder: <i>RoryHaggart_SceneVisibilityInSLAM/MidAir/MidAir</i>. This is the folder that will contain the MidAir dataset
4. From this folder, open a terminal and run the following command to download the data you have selected:

`wget --content-disposition -x -nH -i download_config.txt`
5. Lastly, uncompress all of the downloaded files by running the following command:

`find . -name "*.zip" | while read filename; do unzip -o -d $(dirname "$filename") "$filename"; done;`


# 3. Usage

# 4. Contact
Rory Haggart - [rdeh10@googlemail.com](mailto:rdeh10@googlemail.com)

# Acknowledgements
- ORB-SLAM2 & VINS-Mono creators
- MidAir & Malaga creators
