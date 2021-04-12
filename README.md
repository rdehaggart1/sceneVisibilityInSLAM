# Online Scene Visibility Estimation as a Complement to SLAM in UAVs
As part of the final year research project with the University of Sheffield with the above title, this repository contains supplementary code
to format data and present results. 
## Summary
This project was primarily surrounding simultaneous localisation and mapping (SLAM) systems, and how to estimate the visibility of the scene to a camera in a SLAM system. If the visibility is low, the tracking may decrease in accuracy or be lost entirely. If the visibility can be estimated in some way, the effect that visibility has on the tracking accuracy can be better understood. This could then potentially be used to provide user-facing feedback to take corrective action to improve the visibility, or to develop the path planning strategy in an autonomous system.

Link to final paper:

- [1. Prerequisites](#prereq)
    - [ROS](#ROSPreq)
    - [ORB-SLAM2](#ORB-SLAM2Preq)
    - [VINS-Mono](#VINS-MonoPreq)
    - [Python](#PythonPreq)
        - [Packages](#PyPackPreq)
    - [Notes](#PreqNotes)
- [2. Installation](#install)
    - [Cloning this Repository](#cloningBase)
    - [ORB-SLAM2 and ORB-SLAM3 with Scene Visibility Estimation (SVE)](#ORB2SVE)
        - [Cloning](#cloningORB2)
        - [Adding ROS Package](#packageORB2)
        - [Building](#buildingORB2)
    - [Getting Datasets](#gettingData)
        - [InteriorNet](#getInterior)
        - [MidAir](#getMidair)
    - [Configuring Datasets for Testing](#configData)
        - [InteriorNet](#configInterior)
        - [MidAir](#configMidair)

<a name="prereq"/>

# 1. Prerequisites 

<a name="ROSPreq"/>

## ROS
The method of running datasets for this work was using the [Robot Operating System (ROS)](https://www.ros.org/). This software is only stable for Linux machines, and so it is a soft requirement that this work is only tested on a Linux machine. 

<a name="ORB-SLAM2Preq"/>

## Prerequisites associated with ORB-SLAM2
[ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) is the V-SLAM algorithm that was selected for modifications in support of this project. The modified source code relies on the same dependencies as the original, and so these should be installed as per the original repository instructions.

[Cite paper]

<a name="ORB-SLAM3Preq"/>

## Prerequisites associated with ORB-SLAM3

<a name="PythonPreq"/>

## Python
The files for processing or formatting the datasets in this repository are primarily Python files, and so you will need a Python compiler on your machine to run them.

[version?]

<a name="PyPackPreq"/>

### Python Packages

<a name="PreqNotes"/>

## Notes
The following are some specifications for the setup used during the development process for this project
- Linux Mint 19 ("Tara") Cinnamon partition on a HP Pavilion 15-p209na with Intel© Core™ i3-5010U CPU @ 2.10GHz and 8GB RAM
- ROS Melodic distribution (ROS version 1.4.10)
- Python 3.8.5 compiled with GCC 7.3.0

<a name="install"/>

# 2. Installation

<a name="cloningBase"/>

## Cloning this Repository
To clone this repository onto your machine, open a terminal in the desired cloning location and run the command:

`git clone https://github.com/rdehaggart1/sceneVisibilityInSLAM.git`

<a name="ORB2SVE"/>

## ORB-SLAM2 and ORB-SLAM3 with Scene Visibility Estimation (SVE)
As part of this work, the ORB-SLAM2 and ORB-SLAM3 V- and V-SLAM systems were modified to include scene visibility estimation. The steps for cloning and setting up each are the same, they vary only in the use of the names, <i>ORB-SLAM2</i> and <i>ORB-SLAM3</i>.

<a name="cloningORB2"/>

### Cloning
To access the [modified ORB-SLAM2 source code](https://github.com/rdehaggart1/ORB_SLAM2_SVE), or the [modified ORB-SLAM3 source code](https://github.com/rdehaggart1/ORB_SLAM3_SVE) that include components for scene visibility estimation (SVE), run the corresponding command from the root `sceneVisibilityInSLAM` folder:

```
git clone https://github.com/rdehaggart1/ORB_SLAM2_SVE.git  # ORB-SLAM2 SVE
git clone https://github.com/rdehaggart1/ORB_SLAM3_SVE.git  # ORB-SLAM3 SVE
```

So the path to the modified repository should be `<PATH>/sceneVisibilityInSLAM/ORB_SLAMX_SVE` where X is the ORB-SLAM version.

<a name="packageORB2"/>

### Adding ROS Package

You should now have a local repository of the modified ORB-SLAM(s) code. The original systems each have a ROS package of the same name, and similarly, the modified version has a ROS package of the name <i>ORB_SLAMX_SVE</i>. ROS needs access to this new package, so you must edit your `.bashrc` file to allow for this. This can be done using a text editor like nano to open this file in the following way:

```
cd
nano .bashrc
```

and then moving down to the bottom of the file and pasting the following line (where `<PATH>` is the folder that you cloned <i>sceneVisibilityInSLAM</i> into):

```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:<PATH>/sceneVisibilityInSLAM/ORB_SLAM2_SVE/Examples/ROS # ORB-SLAM2 SVE
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:<PATH>/sceneVisibilityInSLAM/ORB_SLAM3_SVE/Examples/ROS # ORB-SLAM3 SVE
```

Save the changes and exit, and then source your `.bashrc` file to execute the changes:

```
source ./.bashrc
```

To check that the operation was successful, you can restart your terminal to allow the changes to take effect, then run:

```
echo $ROS_PACKAGE_PATH
```

and in the string that is returned, you should see the path to the new package.

<a name="buildingORB2"/>

### Building
Finally, you need to build the modified ORB-SLAMX code for ROS. This means first building the codebase by moving to the root level of the modified `ORB_SLAMX_SVE` repository and running:

```
chmod +x build.sh   # make the build file executable
./build.sh          # execute the file (i.e. build the code)
```

and then running the following to build for use with ROS:

```
chmod +x build_ros.sh
./build_ros.sh
```

Once this has completed, restart your terminal again and run the following to verify that the ORB_SLAMX_SVE package is now available to ROS:

```
rospack list
```

<a name="gettingData"/>

## Getting Datasets
Through the development of this work, two datasets were used for analyses, namely InteriorNet ([website](https://interiornet.org/), [paper](https://interiornet.org/items/interiornet_paper.pdf)) and MidAir ([website](https://midair.ulg.ac.be/), [paper](https://ieeexplore.ieee.org/document/9025697)).

Once you have this repository on your machine and the modified SLAM algorithm(s), you will need some data from the InteriorNet and MidAir datasets to run the code on

<a name="getInterior"/>

### InteriorNet
To download some data for the InteriorNet dataset:
1. Go to the following Google Form to agree to the terms of use and get the dataset link: [https://docs.google.com/forms/d/15sjV-CAud1ENBxK-hJSSrweE7IGc8VXWDbgZiq1trGc](https://docs.google.com/forms/d/15sjV-CAud1ENBxK-hJSSrweE7IGc8VXWDbgZiq1trGc)
2. Click through to the dataset Google Drive folder, which contains <b>a lot</b> of data
3. Now, it's quite difficult to understand what each trajectory contains. Once I have found some that are particularly well suited to this application, I will provide a list here. For now, your best best is to grab some random trajectories from this drive. Each trajectory (e.g. HD5/3FO4KA6US640) contains 3 different extracts each repeated in 'original' and 'random' lighting conditions. 
4. Download the zipped files into the `sceneVisibilityInSLAM/InteriorNet/InteriorNet` folder
5. To get the ground truth for your particular trajectories, you should then go to the `GroundTruth_HD1-HD6` folder and find the .zip archive with the exact same name as the trajectory folder that you just downloaded. The best way to do this is simply to sort the files alphabetically and look for the match (this can be quite tedious, so be prepared!)
6. Download the zipped ground truth into the `sceneVisibilityInSLAM/InteriorNet/InteriorNet` folder, too. You'll need to save this under a different name (e.g. <extract>_groundTruth)
7. Extract both archives. Using 'extract here' will work fine - both archives contain folders with identical names and so all the trajectory data plus the relevant ground truth will automatically move into a single folder. (NOTE: The trajectory archive includes data for multiple cameras (panoramic, fisheye, depth, etc.), extracting all of these could take some time, especially if you're processing multiple trajectories. It might be easier to go into the archive and simply extract the `cam0` folder, as this is the pre-rectified RGB image set and will be the one we use here)
<a name="getMidair"/>

### MidAir
To download some data for the MidAir dataset:
1. Go to [https://midair.ulg.ac.be/download.html](https://midair.ulg.ac.be/download.html)
2. Select all the image types that you are interested in (this code has been tested only on the RGB images), and select the segments from the different environments that you would like to test. <b>Note: the 'Test data for benchmarks' section at the bottom may be a useful starting point as these are short segments that are useful for establishing a good baseline</b>
3. Click <b>Get download links</b> and enter your email address. You will recieve a file containing the links to the data you have selected - download this file into the repository folder: `sceneVisibilityInSLAM/MidAir`.
4. From this folder, open a terminal and run the following command to download the data you have selected:

    ```c
    wget --content-disposition -x -nH -i download_config.txt
    ```
    The dataset will be downloaded into `sceneVisibilityInSLAM/MidAir/MidAir`.

5. Lastly, uncompress all of the downloaded files by running the following command from the same terminal:
    
    ```c
    find . -name "*.zip" | while read filename; do unzip -o -d $(dirname "$filename") "$filename"; done;
    ```

<a name="configData"/>

## Configuring Datasets for Testing

<a name="configInterior"/>

### InteriorNet
Once the data are downloaded and the archives uncompressed, as per the above instructions, the files should be presented as displayed below.
```   
    .
    ├── ...
    ├── InteriorNet                         # All data live in the 'InteriorNet' folder
    │   ├── trajectory                      # The particular trajectory ID (e.g. 3FO4K7IRNF5D, 3FO4KA6US640, ...)
    │       ├── extract                     # The extract within this trajectory with some associated lighting (e.g. original_1_1, random_1_1, original_3_3, ...)
    │           ├── camera                  # The data for different cameras (e.g. cam0, cam0_pano, motion0, ...)
    │               ├── data.csv            # A .csv file containing the filename for each image and the associated timestamp in ns since extract start
    │               ├── data                # The folder containing all the .png images for this extract
    │                   ├── 0000000000031666668.png # The .png image files for the selected extract
    │                   ├── 0000000000071666664.png
    │                   └── ...
    └── ...
```

<a name="configMidair"/>

#### Adding IMU Noise
The provided IMU data in this dataset is by default noiseless, but VI-SLAM algorithms expect some characteristic noise in the inertial sensors. To remedy this, please run `interiornet_addIMUNoise.py`, located at `sceneVisibilityInSLAM/InteriorNet/interiornet_addIMUNoise.py`. The script will ask you to specify the particular trajectory and extract that you'd like to modify the IMU noise for, and ask you what the desired noise characteristics are, and it will go on to add this to the provided values. Remember that if you are making changes to the noise levels in the measurement that you then need to change the corresponding .yaml setup files for the VI- algorithm.

#### Creating a .bag file
As we have described, the format used in the development of this work was ROS .bag files for consistency. To generate a .bag file for a particular InteriorNet extract, please run `interiornet_generateBagFile.py`, located at `sceneVisibilityInSLAM/InteriorNet/interiornet_generateBagFile.py`. The script will ask you to specify the extract of interest, and it will package the images from cam0 (rectified RGB) over this extract under the <b>/camera/image_raw</b> topic. It will also add the IMU data from the corresponding data under the <b>/imu</b> topic. 

[e.g. copying the config files, creating .cc, rebuilding, etc]

### MidAir
Once the data are downloaded and the archives uncompressed, as per the above instructions, the files should be presented as displayed below.
```   
    .
    ├── ...
    ├── MidAir                              # All data live in the 'MidAir' folder
    │   ├── environment                     # The virtual environment map (e.g. Kite_test, VO_test, PLE_training, ...)
    │       ├── condition                   # The condition in the environment (e.g. sunny, winter, foggy, ...)
    │           ├── sensor_records.hdf5     # The .hdf5 dataset file that contains things like IMU data, GPS data, ground truth for each trajectory
    │           ├── camera                  # The onboard camera used (e.g. color_down, color_left)
    │               ├── trajectory          # The trajectory number (e.g. trajectory_0000)
    │                   ├── 000000.JPEG     # The .JPEG image files for the selected extract
    │                   ├── 000001.JPEG
    │                   └── ...
    └── ...
```

#### Generating IMU Data
As a synthetic dataset, the IMU data can be generated based on the ground truth of the test. With the dataset presented as it is immediately after download, some analyses of the `sensor_records.hdf5` file found that the synthesised IMU data was not in a useful format for the use with VI-SLAM algorithms. In particular, the different axes experienced different levels of noise, whereas VI-SLAM algorithms typically expect a single noise standard deviation that is applied across the three axes. In order to address this, [this code presented by the authors](https://github.com/montefiore-ai/midair-dataset/blob/master/tools/IMU-data_generator.py) was modified to produce more consistent IMU data. 

To update the appropriate `sensor_records.hdf5` file which has consistent noise across the three axes, and with this noise value being tuneable within the script, please run `midair_generateIMUData.py`, located at `sceneVisibilityInSLAM/MidAir/midair_generateIMUData.py`. The script will ask you to specify the particular environment and condition that you'd like to generate the data for (note: one `sensor_records.hdf5` file contains all the data for all trajectories in a particular environment and condition)

#### Creating a .bag file
As we have described, the format used in the development of this work was ROS .bag files for consistency. To generate a .bag file for a particular MidAir extract, please run `midair_generateBagFile.py`, located at `sceneVisibilityInSLAM/MidAir/midair_generateBagFile.py`. The script will ask you to specify the environment, trajectory number, condition, and camera that you'd like to generate a bag file for, and it will package the images from this camera over this trajectory under the <b>/camera/image_raw</b> topic. It will also add the IMU data from the corresponding `sensor_records.hdf5` file under the <b>/imu</b> topic. 

[e.g. copying the config files, creating .cc, rebuilding, etc]

# 3. Usage
To run one of the datasets through the modified ORB-SLAM2 algorithm, you have several options

## Run Manually
This method is similar to the method provided by the original ORB-SLAMX documentation. 

Move your terminal to the `ORB_SLAMX_SVE` folder and open 3 terminal tabs.

In the first terminal tab, execute `roscore` to begin the process and allow ROS nodes to communicate with one another:
```
roscore
```

In the second terminal tab, execute `rosrun` to run the executable in the ORB_SLAMX_SVE ROS package, using the provided setup file for the dataset under test. This will setup the algorithm for use and open the visualiser.

For ORB-SLAM2, the command is as shown below:
```
rosrun ORB_SLAM2_SVE Mono Vocabulary/ORBvoc.txt ./Examples/Monocular/interiornet.yaml   # for the InteriorNet dataset
rosrun ORB_SLAM2_SVE Mono Vocabulary/ORBvoc.txt ./Examples/Monocular/midair.yaml        # for the MidAir dataset
```

For ORB-SLAM3, the command is as shown below:
```
rosrun ORB_SLAM3_SVE Mono Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/interiornet.yaml  # for the InteriorNet dataset
rosrun ORB_SLAM3_SVE Mono Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/midair.yaml       # for the MidAir dataset
 ```

And finally, in the third terminal, execute `rosbag play` to playback the .bag file containing all of the dataset extract information, where `<BAG_PATH>` is the full file path to the .bag file of the particular extract of the dataset under test:
```
rosbag play --pause <BAG_PATH>
```

## Run Automatically
For simplicity, it can be useful to perform all of the above operations in a single command

TODO: this may change if i move the .sh file to a common location

Move to the `sceneVisibilityInSLAM/<dataset/ORB-SLAMX_Files` folder, and then run the following commands to make the shell file executable, and then to run the shell file. The result is simply a single command that controls the same processes as in the manual section above 
```
chmod +x <dataset>_ORB-SLAMX.sh     # for example, 'chmod +x midair_ORB-SLAM2.sh' for running midair on ORB-SLAM2
<dataset>_ORB-SLAMX.sh <BAG_PATH>   # for example, 'midair_ORB-SLAM2.sh <PATH>/sceneVisibilityInSLAM/MidAir/MidAir/Kite_training/sunny/trajectory_0021_color_left.bag'
```

## Run Automatically with Results Plotting and Analysis

# 4. Contact
Rory Haggart - [rdeh10@googlemail.com](mailto:rdeh10@googlemail.com)

# Acknowledgements
- **The Authors of ORB-SLAM2:** [Raúl Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan Domingo Tardós Solano](http://webdiis.unizar.es/~jdtardos/), [José María Martínez Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Gálvez-López](http://doriangalvez.com/).
- **The Authors of ORB-SLAM3:**
- **The Authors of the InteriorNet Dataset:**
- **The Authors of the MidAir Dataset:** 
