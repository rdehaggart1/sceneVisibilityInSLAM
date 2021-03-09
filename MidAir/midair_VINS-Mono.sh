#!/bin/bash

#TODO: replace the fixed file names with variables. inputs? or singly defined variables at the top (or in some external setup)

set -m

cd              # go home
cd catkin_ws    # get into the catkin workspace
catkin_make     # build the code that lives there

# setup the workspace
source /home/rory_haggart/catkin_ws/devel/setup.bash

sleep 10        # wait whilst the setup executes

pkill -f vins_  # kill any extra vins processes that are lingering

sleep 10

# launch the vins estimator and use the midair config that's defined in the config .yaml
    # the & at the end moves the process to the background so we can play the bag file on top of it
roslaunch vins_estimator midair.launch & 

# TODO: can we instead 'expect' "no previous pose graph" to trigger the next step? or not bc background process
sleep 20

# a command to write the camera_pose estimate to a .txt file that we can later read
poseCMD="rostopic echo -p /vins_estimator/camera_pose > /media/rory_haggart/ENDLESS_BLU/SLAM_datasets/MidAir/poseGraph.txt"
eval $poseCMD & # again, put this in the background on the main tab

# TODO: switch to mate terminal?
# open a new tab and play the .bag file. --wait holds off on further execution until this tab closes 
gnome-terminal --wait --tab -- rosbag play /media/rory_haggart/ENDLESS_BLU/SLAM_datasets/MidAir/MidAir/Kite_test/sunny/trajectory_0001_color_left.bag

sleep 5         # wait for the estimator to update 

echo -ne "\n\n" # just move the cursor down a couple of lines

sleep 1         # more sleeping

pkill -f vins_  # kill the vins processes again as we're finished now

sleep 5         # let process die

echo -ne "\n\n" # move down to a new prompt

sleep 1

echo -ne "\n\n" # move down to a new prompt
