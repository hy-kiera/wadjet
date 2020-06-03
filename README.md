# wadjet
A capston project in Hanyang Univ.  
Team : By Python  
Team Member  
- [Hayeong Lee](https://github.com/hy-kiera)  
- [Seunghyun Lee](https://github.com/whsqkaak)
- [Kyoungchan Cho](https://github.com/devcre)  

## Environment

### Hardware
- Jetson TX2 (JetPack 4.3)
- DJI F550
- Pixhawk2 Cube 
- RPLidar A1

### Software
- Ubuntu 18.04
- ROS melodic
- PX4 (https://github.com/PX4/Firmware)
- mavros (https://github.com/mavlink/mavros)
- Facenet (https://github.com/davidsandberg/facenet)
- Hector SLAM (https://github.com/NickL77/RPLidar_Hector_SLAM)
- QGroundControl

## How to run

```
$ cd ~/catkin_ws
$ catkin build
$ source devel/setup.bash

$ roslaunch jetson launch.launch
$ roslaunch hector_slam_launch tutorial.launch
$ roslaunch drone_test offb_posctl.launch
```


## rqt_graph

![rqt_graph](https://user-images.githubusercontent.com/17093142/83627690-4c4c3780-a5d2-11ea-9af9-4fd722b09622.png)

#### wadjet/jetson/jetson package
![1](https://user-images.githubusercontent.com/17093142/83627732-60903480-a5d2-11ea-948d-00a2913d7855.png)
Jetson TX2 Onboard Cam is streaming /jetson/video_capture topic.  
The captured video will be processed by face_detector.py  
Then the face points will be subscribed by PixhawkController node.

#### wadjet/jetson/drone_test package
![2](https://user-images.githubusercontent.com/17093142/83627756-671eac00-a5d2-11ea-9423-460311abecbe.png)
All of these topics and nodes are about mavros.  
Finally we control a drone by /posctl_node.

#### wadjet/jetson/RPLidar_Hector_SLAM package
![3](https://user-images.githubusercontent.com/17093142/83627771-6d148d00-a5d2-11ea-8bbb-642f45ff0b90.png)

This package is from that RPLidar_Hector_SLAM link too.
