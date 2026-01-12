#pragma once
#ifndef MAP_H
#define MAP_H

#include "commonInclude.h"
#include "frame.h"
#include "mapPoint.h"

class Map {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFramesType;

    Map(){};

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr mapPoint);
    LandmarksType getAllMapPoints();
    KeyFramesType getAllKeyFrames();

    LandmarksType getActiveMapPoints();
    KeyFramesType getActiveKeyFrames();

    void cleanMap();  // Remove points with zero observations in the map

private:
    void removeOldKeyFrame();

    std::mutex _dataMutex;
    LandmarksType _landmarks;
    LandmarksType _activeLandmarks;
    KeyFramesType _keyframes;
    KeyFramesType _activeKeyFrames;

    Frame::Ptr _currentFrame = nullptr;

    //settings
    unsigned long _numActiveKeyFrames = 15;  // Increased from 7 to keep more map points
};

#endif  // MAP_H