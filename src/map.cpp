#include "map.h"
#include "feature.h"

void Map::insertKeyFrame(Frame::Ptr frame) {
    _currentFrame = frame;
    if (_keyframes.find(frame->_keyframeID) == _keyframes.end()) {
        _keyframes.insert(std::make_pair(frame->_keyframeID, frame));
        _activeKeyFrames.insert(std::make_pair(frame->_keyframeID, frame));
    } else {
        _keyframes[frame->_keyframeID] = frame;
        _activeKeyFrames[frame->_keyframeID] = frame;
    }

    if (_activeKeyFrames.size() > _numActiveKeyFrames)
        removeOldKeyFrame();
}

void Map::insertMapPoint(MapPoint::Ptr mapPoint) {
    if (_landmarks.find(mapPoint->_id) == _landmarks.end()) {
        _landmarks.insert(std::make_pair(mapPoint->_id, mapPoint));
        _activeLandmarks.insert(std::make_pair(mapPoint->_id, mapPoint));
    } else {
        _landmarks[mapPoint->_id] = mapPoint;
        _activeLandmarks[mapPoint->_id] = mapPoint;
    }
}

Map::LandmarksType Map::getAllMapPoints() {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    return _landmarks;
}

Map::KeyFramesType Map::getAllKeyFrames() {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    return _keyframes;
}

Map::LandmarksType Map::getActiveMapPoints() {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    return _activeLandmarks;
}

Map::KeyFramesType Map::getActiveKeyFrames() {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    return _activeKeyFrames;
}

void Map::removeOldKeyFrame() {
    if (_currentFrame == nullptr)
        return;

    double maxDist = 0, minDist = 9999;
    double maxKfID = 0, minKfID = 0;
    auto TWC = _currentFrame->getPose().inverse();

    for (auto& kf : _activeKeyFrames) {
        if (kf.second == _currentFrame)
            continue;
        auto dist = (kf.second->getPose() * TWC).log().norm();

        if (dist > maxDist) {
            maxDist = dist;
            maxKfID = kf.first;
        }
        if (dist < minDist) {
            minDist = dist;
            minKfID = kf.first;
        }
    }

    const double minDistThreshold = 0.2;
    Frame::Ptr frameToRemove = nullptr;
    if (minDist < minDistThreshold)  // remove the most recent frames (within threshold), otherwise remove farthest
        frameToRemove = _keyframes.at(minKfID);
    else
        frameToRemove = _keyframes.at(maxKfID);

    LOG(INFO) << "removed keyframe: " << frameToRemove->_keyframeID;
    _activeKeyFrames.erase(frameToRemove->_keyframeID);
    for (auto feat : frameToRemove->_featuresLeft) {
        if (feat == nullptr)
            continue;
        auto mp = feat->_mapPoint.lock();
        if (mp) {
            mp->removeObservations(feat);
        }
    }
    for (auto feat : frameToRemove->_featuresRight) {
        if (feat == nullptr)
            continue;
        auto mp = feat->_mapPoint.lock();
        if (mp) {
            mp->removeObservations(feat);
        }
    }

    cleanMap();
}

void Map::cleanMap() {
    int landmarksRemovedCount = 0;
    for (auto iter = _activeLandmarks.begin(); iter != _activeLandmarks.end();) {
        if (iter->second->observedTimes == 0) {
            iter = _activeLandmarks.erase(iter);
            landmarksRemovedCount++;
        } else {
            iter++;
        }
    }

    LOG(INFO) << "Removed landmarks: " << landmarksRemovedCount;
}