#pragma once

#ifndef BACKEND_H
#define BACKEND_H

#include "commonInclude.h"

class Backend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    Backend();
};

#endif  // BACKEND_H