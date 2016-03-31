#!/bin/bash
#######################################
#Load Robot client in 3 Steps 
#Step1: set ROS_Master_URI
#Step2: Update RVIZ file
#Step3: Load RVIZ
#######################################

if [ $# != 3 ]  
then 
  echo "USAGE: $0 MasterIP namespace auto_blending" 
  echo " e.g.: $0 192.168.1.20 opnear3 true" 
  exit 0
fi 

#######################################
#paramter
#######################################
ROS_MasterIP=$1
namespace=$2
auto_blending=$3
RVIZSourceFile="irb2400_blending.rviz"
RVIZTargetFile="irb2400_blending_launch.rviz"

#######################################
#Set Master URI address 
#######################################
echo $ROS_MasterIP
export ROS_MASTER_URI="http://"$ROS_MasterIP":11311"
echo $ROS_MASTER_URI


#######################################
#Update Rviz file based on different namespace
#######################################
RVIZFilePath=`rospack find godel_irb2400_support`
RVIZFilePath=$RVIZFilePath"/rviz/"
RVIZSourceFile=$RVIZFilePath$RVIZSourceFile
RVIZTargetFile=$RVIZFilePath$RVIZTargetFile
echo $RVIZSourceFile
echo $RVIZTargetFile
sed "s/TF Prefix: simulation/TF Prefix: ${namespace}/g" $RVIZSourceFile > $RVIZTargetFile 

#######################################
#Update Rviz file based on different namespace
#######################################
roslaunch godel_irb2400_support irb2400_blending_client.launch namespace:=$namespace auto_run:=$auto_blending
