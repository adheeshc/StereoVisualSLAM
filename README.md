# Stereo Visual SLAM

![License](https://img.shields.io/github/license/adheeshc/visual-odometry-cpp)
![Forks](https://img.shields.io/github/forks/adheeshc/visual-odometry-cpp)
![Stars](https://img.shields.io/github/stars/adheeshc/visual-odometry-cpp)
![Issues](https://img.shields.io/github/issues/adheeshc/visual-odometry-cpp)
![Build](https://img.shields.io/badge/build-passing-brightgreen)

This repository implements a customized version of stereo visual SLAM running on the Kitti Dataset.
This SLAM consists of a frontend of optical flow tracking and a backend of sliding window BA.

## Framework

1. <i> bin </i> stores the main file.
2. <i> include </i> stores the header files of the SLAM module.
3. <i> src </i> stores the source code files, mainly .cpp files.
4. <i> test </i> stores the files used for testing.
5. <i> config </i> stores the configuration files.
6. <i> cmake_modules </i> saves the cmake files of third-party libraries, which are used by libraries such as g2o.

## Dataset

- Get the kitti dataset here - https://www.cvlibs.net/datasets/kitti/eval_odometry.php
- Change config/default.yaml to add video sequence

## Dependancies

- [Eigen](https://github.com/libigl/eigen) : Eigen is a C++ template library for linear algebra

- [OpenCV](https://github.com/opencv/opencv) : OpenCV is an C++ library for Image Manipulation

- [Sophus](https://github.com/strasdat/Sophus) : Sophus is an open-source C++ framework for Lie groups commonly used for 2D and 3D geometric problems

- [g2o](https://github.com/RainerKuemmerle/g2o) : g2o is an open-source C++ library for optimizing graph-based nonlinear error functions

- [Glog](https://github.com/google/glog) : C++ implementation of the Google logging module

- [Gflags](https://github.com/gflags/gflags) : The gflags package contains a C++ library that implements command line flags processing

- [Gtest](https://github.com/google/googletest) : Google Testing and Mocking Framework

- [Pangolin](https://github.com/stevenlovegrove/Pangolin) : Pangolin is a lightweight portable rapid development library for managing OpenGL display / interaction

## Build

- Check if all dependancies are handled
- Update dataset path in config/default.yaml

```
mkdir -p build
cd build
cmake ..
make
./runKittiStereo
```
