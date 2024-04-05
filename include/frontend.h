#pragma once
#ifndef FRONTEND_H
#define FRONTEND_H

#include "camera.h"
#include "commonInclude.h"
#include "frame.h"
#include "map.h"

class Backend;
class Viewer;

enum class FrontEndStatus { INITIALIZE,
                            TRACKING_GOOD,
                            TRACKING_BAD,
                            TRACKING_LOST };

// Estimate the current frame Pose, add keyframes to the map and trigger optimization when the keyframe conditions are met.
class Frontend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();
};

#endif  // FRONTEND_H