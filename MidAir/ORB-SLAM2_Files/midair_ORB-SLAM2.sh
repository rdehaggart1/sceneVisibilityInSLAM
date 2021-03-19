#!/bin/bash

set -m

killall -9 roscore # kill any roscore/master processes that are lingering
killall -9 rosmaster

sleep 10

# launch roscore
    # the & at the end moves the process to the background so we can play the bag file on top of it
roscore & 

# TODO: can we instead 'expect'
sleep 30

# a command to write the camera_pose estimate to a .txt file that we can later read
# TODO: verify that the simple 'poseGraph.txt' works
#BASEDIR=BASEDIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
#poseCMD="rostopic echo -p /vins_estimator/camera_pose > $BASEDIR/poseGraph.txt"
#eval $poseCMD & # again, put this in the background on the main tab

poseCMD="rosrun ORB_SLAM2_SVE Mono /media/rory_haggart/ENDLESS_BLU/sceneVisibilityInSLAM/ORB_SLAM2_SVE/Vocabulary/ORBvoc.txt /media/rory_haggart/ENDLESS_BLU/sceneVisibilityInSLAM/ORB_SLAM2_SVE/Examples/Monocular/midair.yaml"
eval $poseCMD &

sleep 20

# TODO: switch to mate terminal?
# open a new tab and play the .bag file. --wait holds off on further execution until this tab closes 
gnome-terminal --wait --tab -- rosbag play $1 /cam0/image_raw:=/camera/image_raw

sleep 5         # wait for the estimator to update 

echo -ne "\n\n" # just move the cursor down a couple of lines

sleep 1         # more sleeping

killall -9 roscore      # kill any roscore/master processes that are lingering
killall -9 rosmaster

sleep 5         # let process die


processID=$(pidof Mono)
kill -SIGINT $processID
#pkill -f Mono

echo -ne "\n\n" # move down to a new prompt

sleep 1

echo -ne "\n\n" # move down to a new prompt
