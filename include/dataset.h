#pragma once

#ifndef DATASET_H
#define DATASET_H

#include "camera.h"
#include "commonInclude.h"
#include "frame.h"

class Dataset {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;

    Dataset(const std::string& _path);
    bool init();                                // Initialization, returns whether successful
    Frame::Ptr nextFrame();                     // Create and return the next frame containing the stereo images
    Camera::Ptr getCamera(int cameraID) const;  // Get camera by id

private:
    std::string _path;
    int _currentImgIndex = 0;

    std::vector<Camera::Ptr> _cameras;
};

#endif  // DATASET_H