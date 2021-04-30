#!/bin/bash

set -m

killall -9 roscore # kill any roscore/master processes that are lingering
killall -9 rosmaster
killall -9 rosout

sleep 5

# launch roscore
    # the & at the end moves the process to the background so we can play the bag file on top of it
roscore & sleep 10

# a command to run ORB_SLAM2_SVE with the midair setup
poseCMD="rosrun ORB_SLAM3_SVE Mono ../../ORB_SLAM3_SVE/Vocabulary/ORBvoc.txt ../../ORB_SLAM3_SVE/Examples/Monocular-Inertial/midair.yaml"

eval $poseCMD &

sleep 20

# TODO: switch to mate terminal?
# open a new tab and play the .bag file. --wait holds off on further execution until this tab closes 
gnome-terminal --wait --tab -- rosbag play $1

sleep 5         # wait for the estimator to update 

echo -ne "\n\n" # just move the cursor down a couple of lines

sleep 1         # more sleeping

killall -9 roscore      # kill any roscore/master processes that are lingering
killall -9 rosmaster
killall -9 rosout

sleep 2         # let process die

processID=$(pidof Mono)
kill -SIGINT $processID

echo -ne "\n\n" # move down to a new prompt

sleep 1

echo -ne "\n\n" # move down to a new prompt
