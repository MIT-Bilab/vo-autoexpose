# vo-autoexpose

**Author:** Marc-André Bégin

<!-- The [Changelog](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md) describes the features of each version. -->

This repository contains the source code of an auto-exposure algorithm for maxing out VO performance in challenging light conditions. A ROS wrapper is also provided for the algorithm and the whole repository is structured as a ROS package. The core algorithm library is however independant of ROS and can be reused directly.

<!-- We provide examples to run ORB-SLAM3 in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) using stereo or monocular, with or without IMU, and in the [TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) using fisheye stereo or monocular, with or without IMU. Videos of some example executions can be found at [ORB-SLAM3 channel](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q). -->

This software reuses some of the code from [aer_auto_exposure_gradient](https://github.com/ishaanmht/aer_auto_exposure_gradient) developped by [Ishaan Mehta, Tang Mingliang, and Timothy D. Barfoot](https://ieeexplore.ieee.org/document/9108676/). 
<!-- This software is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) developed by [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)). -->

<!-- TODO: Include a cool picture -->
<!-- <a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://img.youtube.com/vi/HyLNq-98LRo/0.jpg" 
alt="ORB-SLAM3" width="240" height="180" border="10" /></a> -->


# 1. License

vo-autoexpose is released under [MIT license](https://github.com/MIT-Bilab/vo-autoexpose/blob/main/LICENSE). 

If you use this code in an academic work, please cite:

    @article{vo_autoexpose,
      title={Auto-Exposure Algorithm for Enhanced Mobile Robot Localization in Challenging Light Conditions},
      author={B\´egin, Marc-Andr\´e AND Other, Authors.},
      journal={MDPI Sensors Journal},
      year={2021}
     }

# 2. Prerequisites
The library was tested in **Ubuntu 18.04**. 

<!-- ## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11. -->

## OpenCV
Dowload and installation instructions can be found at: http://opencv.org. **Requires at least 3.2.0 (default with ROS melodic installation)**.

## Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org. **Requires at least 3.1.0**.

## ROS 
The algorithm was tested with ROS Melodic. Follow download and installation instructions at: http://wiki.ros.org/melodic.

## Camera driver
The ROS wrapper for the auto-exposure algorithm included in this repo assumes that you are using a Pt Grey (FLIR) camera supported by Spinnaker SDK. If you are using another camera, you should only need to update the "send_params" and "image_cb" callback functions in "vo_autoexpose_node.cpp" to send the acquisition parameters to the camera and to pull new frames.

Download and installation instructions for Spinnaker SDK can be found at: https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-and-firmware-download/.

This repository includes a ROS package with wrappers for the Spinnaker SDK which was forked from the [spinnaker_sdk_camera_driver](https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver) repository. It was modified to support real-time alterations to the camera exposure gain and to publish ROS messages containing the exposure time and gain of each captured frame. If you are using your own camera drivers, you can safely delete the whole folder named "spinnaker_sdk_camera_driver" to avoid building the package.

# 3. Building vo-autoexpose library and ROS wrapper
Move to the source folder of your catkin repository :
```
cd catkin_rep/src
```

Clone the repository:
```
git clone https://github.com/MIT-Bilab/vo-autoexpose vo_autoexpose
```
Build:
```
cd ..
catkin build
```
Upon successful build, do not forget to source:
```
source devel/setup.bash 
```

# 4. Usage
To start the auto-exposure controller:
```
roslaunch vo_autoexpose vo_autoexpose.launch
```
Of course, nothing will happen until you start streaming images from the cameras. 

<!-- TODO: add details on https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver -->
