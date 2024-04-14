#pragma once

#ifndef SLAM_H
#define SLAM_H

#include "backend.h"
#include "camera.h"
#include "commonInclude.h"
#include "dataset.h"
#include "frontend.h"
#include "viewer.h"

class SLAM {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<SLAM> Ptr;

    SLAM(std::string& configPath);

    bool Initialize();  // Initialization
    void Run();         // Run SLAM on dataset
    bool Step();        // Take a step forward in dataset

    FrontEndStatus getFrontEndStatus();

private:
    bool _initialized = false;
    std::string _configFilePath;

    Frontend::Ptr _frontend = nullptr;
    Backend::Ptr _backend = nullptr;
    Map::Ptr _map = nullptr;
    Viewer::Ptr _viewer = nullptr;

    Dataset::Ptr _dataset = nullptr;  // dataset
};

#endif  // SLAM_H