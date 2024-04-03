# Stereo Visual SLAM

This repository implements a customized version of stereo visual SLAM running on the Kitti Dataset.
This SLAM consists of a frontend of optical flow tracking and a backend of sliding window BA.

## Framework
1. <i> bin </i> stores the compiled binary file
2. <i>include/myslam</i>  stores the header files of the SLAM module.
3. <i>src</i> stores the source code files, mainly .cpp files.
4. <i>test</i> stores the files used for testing, which are also .cpp files.
5. <i>config</i> stores the configuration files.
6. <i>cmake_modules</i> saves the cmake files of third-party libraries, which are used by libraries such as g2o.

## Dataset

get the kitti dataset here - https://www.cvlibs.net/datasets/kitti/eval_odometry.php
change config/default.yaml to add video sequence

