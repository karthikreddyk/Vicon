<script type="text/javascript" async
  src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-MML-AM_CHTML">
</script>

#Vicon Listener
##Subscriber to the ROS VICON_SDK

##Author: [Olalekan Ogunmolu](http://lakehanne.github.io)

## Introduction

This code implements a roscpp subscriber to the [**bridge**](http://wiki.ros.org/vicon_bridge) of [**Vicon SDK**](http://www.vicon.com/downloads/utilities-and-sdks/datastream-sdk/vicon-datastream-sdk-15) with **ROS**.

## Overview

The project pipeline is as follows:

- The inputs are the sets of markers on your subject.
- The output from `vicon_bridge` is the `frame_id`, `header stamp` and `markers attributes` from `vicon_sdk` API
- `Vicon_listener` subscribes via [ROS class methods](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers) to `vicon_bridge` 
- The marker points are packed into a `struct` called `headmarker`. The struct header file is in the directory:` [vicon_listener/structs.h](https://github.com/lakehanne/vicon_listener/blob/master/include/vicon_listener/structs.h)
- To avoid numerical instability, we compute the modified Gram-Schmidt orthogonal and orthonormal basis.
	
	- See   `https://en.m.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process#CITEREFGolubVan_Loan1996`	and `http://www.cs.cmu.edu/~kiranb/animation/p245-shoemake.pdf*/`
	- The othogonal and orthonormal basis are stored in the `struct` `gonal` and `normal` respectively.

- Retrieving the rotation matrix is trivial after we have the matrix of orthonormal basis of each marker's vector: simply pick any three orthonormal basis and concatenate them into an `Eigen MatrixXd`. See the 	function `void rot(orth normal)` within the `vicon_listener` `src` directory.

- Even though the documentation says Vicon is a right-handed axis frame system, we found that the retrieved marker points are left-handed (i.e. det(R) = -1). To make \\(R \in SO(3)\\), we simply remap the left-hand vectors into right-hand vectors.


## 	Streaming pose
- 	Calibrate your subjects's origin; then add a `new subject model template` by following the video [here](http://hci.rwth-aachen.de/vicon_nexus_tutorials).
-	Clone  vicon_bridge from the github repo [vicon bridge](https://github.com/ethz-asl/vicon_bridge) to your `catkin_ws/src` folder: `git clone https://github.com/ethz-asl/vicon_bridge`.
- 	The `vicon_bridge` node initiates a connection with the Vicon data source (e.g. Nexus or Tracker) with the Datastream API. So, set the ip address of your client system to that of the datastream server (e.g. 192.168.1.7:801). Do this within `~/catkin_ws/src/vicon_bridge/launch/vicon.launch.
	Note, default port number **is** 801.
-	Clone `vicon_listener` into your `catkin_ws/src` folder: `git clone https://github.com/lakehanne/vicon_listener.git`
-	Compile both `vicon_listener` and `vicon_bridge` with `catkin_make --source src/vicon_bridge --source src/vicon_listener`
-	Launch the `ROS_bridge` to `Vicon_SDK` within `catkin_ws: `rosrun vicon_bridge vicon_bridge`
-	In a separate terminal, do `cd ~/catkin_ws` and `rosrun vicon_listener vicon_listener`.






