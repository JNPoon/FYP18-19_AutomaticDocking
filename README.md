# FYP18/19- Automatic Docking and Charging Station for A Mobile Robot Platform
## Overview
This is a Final Year Project. The objective is to enable to mobile robot to locate the station and dock successfully for charging. 
`Please rename the package name to docking`

The robot use a holonomic base platform and the peripherals used for perception are monochrome camera and Hokuyo URG-04LX Laser Range Finder.

This repo consists of 2 main nodes: 
- Docking Algorithm
- Laser Check Algorithm

## Prerequisites
### Packages
4 packages to be installed for perception
- [usb_cam](http://wiki.ros.org/usb_cam)
- [image_view](http://wiki.ros.org/image_view)
- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)
- [hokuyo_node](http://wiki.ros.org/hokuyo_node)
### Odometry
An odom node is required. The odom node should publish a topic named `/chargingvoltage` to detect the battery is charging.
If not, a node is required to publish the `/chargingvoltage` topic

## Usage
### Docking Algorithm
Run the docking node with: 
```
roslaunch docking final_docking_alvar.launch
```
### Laser Check Algorithm
Run the laser docked check node with:
```
roslaunch docking final_laser_check.launch
```

## Launch Files
### 1. final_docking_alvar.launch
This will launch the `usb_cam node`, the `image_view node`, the `ar_track_alvar node`, and the `final_docking_algorithm node`. 
#### Parameters
- `zError` - The error between z_0 and z_1 when the robot is perfectly parallel to the station, in meter.
  - Default: `0.03`
- `ReverseDistance` - The distance for the reverse function to reach when it is triggered, in meter.
  - Default: `0.65`
- `CheckPointCount` - The initial value to count down to 0 when the robot is at the check point area.
  - Default: `50`
### 2. final_laser_check.launch
This will launch  the `hokuyo node`, and the `laser_docked_check node`. 
#### Parameters
- `OrientValue` - A benchmark to compare with the number of matches between mid-point distance and all other distances 
during orientation check.
  - Default: `17`
- `Tolerance` - A tolerance for the comparison between the mid-point distance and all other distances 
when checking orientation using the laser, in meter.
  - Default: `0.015`
- `DockedDistance` - The distance from the laser to the station when the robot is docked, for comparison 
when doing the second check, in meter.
  - Default: `0.178`
> For other parameters in other nodes from other packages, please refer to their documentation 
[here](https://github.com/JNPoon/FYP18-19_AutomaticDocking#packages)

## Nodes
### 1. final_docking_algorithm node
Get distance value from ar_track_alvar and perform motion planning
#### Subscribed Topics
- `/ar_pose_marker_Small` (ar_track_alvar_msgs/AlvarMarkers)  
  The distance value of the marker from the camera
- `/chargingvoltage` (std_msgs/Float32)  
  The voltage of the battery during charging state and non-charging state
- `/FinalCheck` (docking/FinalCheck)  
  The messages to communicate with laser_docked_check node
#### Published Topics
- `/cmd_vel` (geometry_msgs/Twist)  
  The velocity to be sent to motion controller by odom node  
- `/FinalCheck` (docking/FinalCheck)  
  The messages to communicate with laser_docked_check node
  
### 2. laser_docked_check node
Check the orientation at check point and whether docked successfully
#### Subscribed Topics
- `/scan1` (sensor_msgs/LaserScan)  
  The distance value from the Hokuyo Laser Range Finder  
- `/chargingvoltage` (std_msgs/Float32)  
  The voltage of the battery during charging state and non-charging state
- `/FinalCheck` (docking/FinalCheck)  
  The messages to communicate with final_docking_algorithm node
#### Published Topics
- `/FinalCheck` (docking/FinalCheck)  
  The messages to communicate with final_docking_algorithm node
  
## Details Description
For more detail documentation, please refer to [here](documentation/Draft_FinalReport_Heading.docx)
  
  
  
  
