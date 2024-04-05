#pragma once

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// std
#include <algorithm>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
namespace mySlam {
    typedef Eigen::Matrix<double, 2, 1> Vec2;
    typedef Eigen::Matrix<double, 3, 1> Vec3;
    typedef Eigen::Matrix<double, 3, 3> Mat33;
};

// Sophus
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

// OpenCV
#include <opencv2/opencv.hpp>

// glog
#include <glog/logging.h>

#endif  // COMMON_INCLUDE_H