#pragma once

#ifndef FEATURE_H
#define FEATURE_H

#include "commonInclude.h"

struct Frame;
struct MapPoint;

struct Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> _frame;        // The frame that uses this feature
    cv::KeyPoint _position;             // 2D pixel position
    std::weak_ptr<MapPoint> _mapPoint;  // Assigned map point

    bool _isOutlier = false;    // check if outlier
    bool _isOnLeftImg = false;  // check if feature is detected on left img
public:
    Feature(){};
    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint position);
};

#endif  // FEATURE_H