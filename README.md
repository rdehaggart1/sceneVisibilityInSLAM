# Online Scene Visibility Estimation as a Complement to SLAM in UAVs
As part of the final year research project with the University of Sheffield with the above title, this repository contains supplementary code
to format data and present results. 
## Summary
This project was primarily surrounding simultaneous localisation and mapping (SLAM) systems, and how to estimate the visibility of the scene to a camera in a SLAM system. If the visibility is low, the tracking may decrease in accuracy or be lost entirely. If the visibility can be estimated in some way, the effect that visibility has on the tracking accuracy can be better understood. This could then potentially be used to provide user-facing feedback to take corrective action to improve the visibility, or to develop the path planning strategy in an autonomous system.

Link to research paper:

# 1. Prerequisites 
## ROS
## ORB-SLAM2 (and associated prerequisites)
[ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) is the V-SLAM algorithm that was selected for modifications in support of this project. 
[Cite paper]
## VINS-Mono (and associated prerequisites)
## Python
## Packages

# 2. Installation
## Cloning
To clone this repository onto your machine, open a terminal in the desired cloning location and run the command:

`git clone https://github.com/rdehaggart1/sceneVisibilityInSLAM.git`

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
- **The Authors of ORB-SLAM2:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan Domingo Tardós Solano](http://webdiis.unizar.es/~jdtardos/), [José María Martínez Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/).
- **The Authors of VINS-Mono:**
- **The Authors of the MidAir Dataset:**
- **The Authors of the Malaga Dataset:** [Jose-Luis Blanco-Claraco](https://scholar.google.co.uk/citations?user=bhDtzKgAAAAJ&hl=en), [Francisco-Angel Moreno-Duenas](http://mapir.isa.uma.es/mapirwebsite/index.php/people/199-francisco-moreno-due%C3%B1as) and [Javier Gonzalez-Jimenez](http://mapir.isa.uma.es/mapirwebsite/index.php/people/95-javier-gonzalez-jimenez)
