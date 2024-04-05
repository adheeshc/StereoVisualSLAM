#pragma once

#ifndef DATASET_H
#define DATASET_H

#include "commonInclude.h"

class Dataset {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;

    Dataset(const std::string& path);
};

#endif  // DATASET_H