#include <iostream>

#include "frame.h"

Frame::Frame(long id, double timeStamp, const SE3& pose, const cv::Mat& left, const cv::Mat& right)
    : _id(id), _timeStamp(timeStamp), _pose(pose), _leftImg(left), _rightImg(right) {}

Frame::Ptr Frame::createFrame() {
    static long factoryID = 0;
    Frame::Ptr newFrame(new Frame);
    newFrame->_id = factoryID++;
    return newFrame;
}

void Frame::setKeyframe() {
    static long keyframeFactoryID = 0;
    _isKeyframe = true;
    _keyframeID = keyframeFactoryID++;
}