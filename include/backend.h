#pragma once

#ifndef BACKEND_H
#define BACKEND_H

#include "camera.h"
#include "commonInclude.h"
#include "frame.h"
#include "map.h"

class Map;
class Backend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    Backend();

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        _leftCamera = left;
        _rightCamera = right;
    }

    void SetMap(std::shared_ptr<Map> map) {
        _map = map;
    }

    void updateMap();
    void Stop();

private:
    void BackendLoop();

    void Optimize(Map::KeyFramesType& keyframes, Map::LandmarksType& landmarks);

    std::shared_ptr<Map> _map;
    std::thread _backendThread;
    std::mutex _dataMutex;

    std::condition_variable _mapUpdate;
    std::atomic<bool> _backendRunning;

    Camera::Ptr _leftCamera = nullptr;
    Camera::Ptr _rightCamera = nullptr;
};

#endif  // BACKEND_H