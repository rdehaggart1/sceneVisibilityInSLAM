# Online Scene Visibility Estimation as a Complement to SLAM in UAVs
As part of the final year research project with the University of Sheffield with the above title, this repository contains supplementary code
to format data and present results. 
## Summary
This project was primarily surrounding simultaneous localisation and mapping (SLAM) systems, and how to estimate the visibility of the scene to a camera in a SLAM system. If the visibility is low, the tracking may decrease in accuracy or be lost entirely. If the visibility can be estimated in some way, the effect that visibility has on the tracking accuracy can be better understood. This could then potentially be used to provide user-facing feedback to take corrective action to improve the visibility, or to develop the path planning strategy in an autonomous system.

Link to research paper:

# 1. Prerequisites 
## ROS
## ORB-SLAM2 associated prerequisites)
[ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) is the V-SLAM algorithm that was selected for modifications in support of this project. The modified source code relies on the same dependencies as the original, and so these should be installed as per the original repository instructions. <b>Note: please install these dependencies to your <i>Home</i> directory as they should be useful for more than just this project!</b>

[Cite paper]
## VINS-Mono (and associated prerequisites)
## Python
## Packages

# 2. Installation
## Cloning the Base Repository
To clone this repository onto your machine, open a terminal in the desired cloning location and run the command:

`git clone https://github.com/rdehaggart1/sceneVisibilityInSLAM.git`

## Cloning ORB-SLAM2 with Scene Visibility Estimation
To access the [modified ORB-SLAM2 source code](https://github.com/rdehaggart1/ORB_SLAM2_withVisibilityEstimation) that includes components for scene visibility estimation, move to the location that you'd like to clone the modified ORB-SLAM2 code into, and clone by running the following commands:

```
cd <PATH>
git clone https://github.com/rdehaggart1/ORB_SLAM2_withVisibilityEstimation.git
```

## Preparing to use ORB-SLAM2 with Scene Visibility Estimation

You should now have a local repository of the modified ORB-SLAM2 code. The original ORB-SLAM2 has a ROS package of the same name, and similarly, the modified version has a ROS package of the name 'ORB_SLAM2_withVisibilityEstimation'. ROS needs access to this new package, so you must edit your <i>.bashrc</i> file to allow for this. This can be done using a text editor like nano to open this file in the following way:

```
cd
nano .bashrc
```

and then moving down to the bottom of the file and pasting the following line (where `<PATH>` is the folder that the <i>ORB_SLAM2_withVisibilityEstimation</i> repository lives in, the same as the one that you `cd`'d to above):

```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:<PATH>/ORB_SLAM2_withVisibilityEstimation/Examples/ROS
```

Save the changes and exit, and then source your <i>.bashrc</i> file to execute the changes:

```
source ./.bashrc
```

To check that the operation was successful, you can restart your terminal to all the changes to take effect, then run:

```
echo $ROS_PACKAGE_PATH
```

and in the string that is returned, you should see the path that you just added to the new package.

## Building ORB-SLAM2 with Scene Visibility Estimation
Finally, you need to build the modified ORB-SLAM2 code for ROS. This means first building the codebase by moving to the base level of the modified ORB-SLAM2 repository and running:

```
chmod +x build_ros.sh
./build.sh
```

and then running the following to build for use with ROS:

```
chmod +x build_ros.sh
./build_ros.sh
```

Once this has completed, restart your terminal again and run:

```
rospack list
```

to verify that the ORB_SLAM2_withVisibilityEstimation package is now avaialable to ROS

## Getting Datasets
Once you have the repository on your machine, you will need some data from the Malaga and MidAir datasets to run the code on
### Malaga
To download some data for the Malaga dataset:
1. Go to [https://www.mrpt.org/MalagaUrbanDataset](https://www.mrpt.org/MalagaUrbanDataset)
2. Use this page to browse the previews of the various extracts and download the .zip files that you are interested in to the repository folder: <i>sceneVisibilityInSLAM/Malaga/Malaga</i>. This is the folder that will contain the Malaga dataset
3. Lastly, uncompress the files to the same location
### MidAir
To download some data for the MidAir dataset:
1. Go to [https://midair.ulg.ac.be/download.html](https://midair.ulg.ac.be/download.html)
2. Select all the image types that you are interested in (this code has been tested only on the RGB images), and select the segments from the different environments that you would like to test. <b>Note: the 'Test data for benchmarks' section at the bottom may be a useful starting point as these are short segments that are useful for establishing a good baseline</b>
3. Click <b>Get download links</b> and enter your email address. You will recieve a file containing the links to the data you have selected - download this file into the repository folder: <i>sceneVisibilityInSLAM/MidAir/MidAir</i>. This is the folder that will contain the MidAir dataset
4. From this folder, open a terminal and run the following command to download the data you have selected:

    ```c
    wget --content-disposition -x -nH -i download_config.txt`
    ```

5. Lastly, uncompress all of the downloaded files by running the following command from the same terminal:
    
    ```c
    find . -name "*.zip" | while read filename; do unzip -o -d $(dirname "$filename") "$filename"; done;`
    ```

## Configuring Datasets for Use with the Algorithms
[e.g. copying the config files, creating .cc, rebuilding, etc]

# 3. Usage

# 4. Contact
Rory Haggart - [rdeh10@googlemail.com](mailto:rdeh10@googlemail.com)

# Acknowledgements
- **The Authors of ORB-SLAM2:** [Raúl Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan Domingo Tardós Solano](http://webdiis.unizar.es/~jdtardos/), [José María Martínez Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Gálvez-López](http://doriangalvez.com/).
- **The Authors of VINS-Mono:**
- **The Authors of the MidAir Dataset:**
- **The Authors of the Malaga Dataset:** [José-Luis Blanco-Claraco](https://scholar.google.co.uk/citations?user=bhDtzKgAAAAJ&hl=en), [Francisco-Ángel Moreno-Dueñas](http://mapir.isa.uma.es/mapirwebsite/index.php/people/199-francisco-moreno-due%C3%B1as) and [Javier González-Jiménez](http://mapir.isa.uma.es/mapirwebsite/index.php/people/95-javier-gonzalez-jimenez)
