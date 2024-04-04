#include <iostream>

#include "feature.h"

Feature::Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint position)
    : _frame(frame), _position(position) {}