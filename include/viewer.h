#pragma once

#ifndef VIEWER_H
#define VIEWER_H

#include "commonInclude.h"

class Viewer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();
};

#endif // VIEWER_H