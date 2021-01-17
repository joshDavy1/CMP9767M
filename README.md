# CMP9767M

## Overview
My focus was on the Simulation specialistion. For my work I have implemented a realistic precision sprayer for thorvald. This was designed in Solidworks and a exporter was used to create the URDF file. This was then intergrated in the existing thorvald description. PID JointEffortControllers are then setup to control the joints of the arm. Then a inverse kinematic solution was solved by hand and implemented in thorvald_arm. A simple HSV filter based weed detector is also implemented and a master node controls the arm and thorvald to move along rows and spray weeds.

## Install 
Install ros melodic and setup up LCAS distribution:
https://github.com/LCAS/CMP9767M/wiki/useful-resources#install-the-l-cas-ubuntu-distribution-for-the-module-on-your-own-pc

The following ROS packages are required:
```
sudo apt-get install ros-melodic-effort-controllers ros-melodic-find-object-2d ros-melodic-gmapping ros-melodic-image-geometry ros-melodic-image-view ros-melodic joint-state-publisher-gui ros-melodic-kinect2-description ros-melodic-opencv-apps ros-melodic-robot-localization ros-melodic-robot-pose-publisher ros-melodic-rqt-image-view ros-melodic-teleop-tools ros-melodic-thorvald ros-melodic-topological-navigation ros-melodic-topological-utils ros-melodic-urdf-tutorial ros-melodic-velodyne-description ros-melodic-video-stream-opencv
```

Note this uses a custom version of uol_cmp8767m_base. Any other version of this installed or in your catkin_ws must be removed first.

The reposistory can then be cloned to your catkin_ws.

```
cd ~/catkin_ws
git clone https://github.com/joshDavy1/CMP9767M
catkin_make
```

## Run
First edit the crop row start and end coords appropiately
```
nano ~/catkin_ws/CMP9767M/master_node/config/crop_row_coords.yaml
```
Start the simulation and Rviz
``` 
roslaunch master_node start.launch
```
Run
```
rosrun master_node mover.py
```
