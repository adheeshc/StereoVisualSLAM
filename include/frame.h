#ifndef FRAME_H
#define FRAME_H

#include "commonInclude.h"

struct MapPoint;
struct Feature;

// Each frame is assigned an independent ID, and key frames are assigned key frame IDs.
struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long _id = 0;         // frame ID
    unsigned long _keyframeID = 0;  // keyframe ID
    bool _isKeyframe = false;      // check if keyframe
    double _timeStamp;             // timestamp
    SE3 _pose;                     // pose as TCW
    std::mutex _poseMutex;         // pose data mutex
    cv::Mat _leftImg, _rightImg;   // stereo images

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> _featuresLeft;
    // extracted features in right image
    std::vector<std::shared_ptr<Feature>> _featuresRight;

public:  // data members
    Frame(){};
    Frame(long id, double timeStamp, const SE3& pose, const cv::Mat& left, const cv::Mat& right);

    // set and get pose, thread safe
    SE3 getPose() {
        std::unique_lock<std::mutex> lck(_poseMutex);
        return _pose;
    }

    void setPose(const SE3& pose) {
        std::unique_lock<std::mutex> lck(_poseMutex);
        _pose = pose;
    }

    // set keyframe and keyframe ID
    void setKeyframe();

    // create new frame and allocate ID
    static std::shared_ptr<Frame> createFrame();
};

#endif  // FRAME_H
