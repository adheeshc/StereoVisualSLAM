#pragma once

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "commonInclude.h"

inline bool triangulation(const std::vector<SE3>& poses, const std::vector<mySlam::Vec3> points, mySlam::Vec3& ptWorld) {
    mySlam::MatXX A(2 * poses.size(), 4);
    mySlam::VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); i++) {
        mySlam::Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    ptWorld = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        return true;
    }

    return false;
}

inline mySlam::Vec2 toVec2(const cv::Point2f p) {
    return mySlam::Vec2(p.x, p.y);
}

#endif  // ALGORITHM_H