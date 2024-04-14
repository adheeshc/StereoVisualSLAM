#pragma once
#ifndef FRONTEND_H
#define FRONTEND_H

#include "camera.h"
#include "commonInclude.h"
#include "frame.h"
#include "map.h"
#include "feature.h"

class Backend;
class Viewer;

enum class FrontEndStatus { INITIALIZE,
                            TRACKING_GOOD,
                            TRACKING_BAD,
                            TRACKING_LOST,
                            TESTING };

// Estimate the current frame Pose, add keyframes to the map and trigger optimization when the keyframe conditions are met.
class Frontend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    bool addFrame(Frame::Ptr frame);
    void setMap(Map::Ptr map);
    void setBackend(std::shared_ptr<Backend> backend);
    void setViewer(std::shared_ptr<Viewer> viewer);
    void setCameras(Camera::Ptr left, Camera::Ptr right);
    FrontEndStatus getFrontEndStatus();

private:
    FrontEndStatus _status = FrontEndStatus::INITIALIZE;
    std::shared_ptr<Backend> _backend;
    std::shared_ptr<Viewer> _viewer;

    Frame::Ptr _currentFrame = nullptr;
    Frame::Ptr _lastFrame = nullptr;
    Camera::Ptr _leftCamera = nullptr;
    Camera::Ptr _rightCamera = nullptr;

    Map::Ptr _map = nullptr;

    SE3 _relativeMotion;  // relative motion between the currentFrame and the previous frame is used to estimate the initial pose value of the current frame.

    int _trackingInliers = 0;  // for testing new keyframes

    // params
    int _numFeatures = 200;
    int _numFeaturesInit = 100;
    int _numFeaturesTracking = 50;
    int _numFeaturesTrackingBad = 20;
    int _numFeaturesNeededForKeyFrame = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> _gftt;    // Feature Detector

    bool Track();                       // Track in normal mode
    bool Reset();                       // Reset when lost
    int trackLastFrame();               // Track with last frame
    int estimateCurrentPose();          // Estimate current frame's pose
    bool insertKeyFrame();              // Set current frame as KeyFrame and insert it into backend
    bool stereoInit();                  // Try to initialize with stereo images saved in _currentFrame
    int detectFeatures();               // Detect features in left image in  _currentFrame
    int findFeaturesInRight();          // Find corresponding features in right image in _currentFrame
    bool buildInitMap();                // Build initial map with single image
    int triangulateNewPoints();         // Triangulate the 2D points in _currentFrame
    void setObservationsForKeyFrame();  // Set the features in keyframe as new observation of the map points
};

#endif  // FRONTEND_H