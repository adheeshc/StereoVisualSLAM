#pragma once

#ifndef CAMERA_H
#define CAMERA_H

#include "commonInclude.h"

class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    double _fx = 0;  // Camera intrinsics
    double _fy = 0;
    double _cx = 0;
    double _cy = 0;
    double _baseline = 0;

    SE3 _pose;     // Extrinsic, from stereo camera to single camera
    SE3 _poseInv;  // Inverse of extrinsics

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline, const SE3& pose);

    SE3 getPose() const;
    mySlam::Mat33 getIntrinsics() const;

    // camera transform between world, camera and pixel
    mySlam::Vec3 world2Camera(const mySlam::Vec3& pw, const SE3& TCW);
    mySlam::Vec3 camera2World(const mySlam::Vec3& pc, const SE3& TCW);

    mySlam::Vec2 camera2Pixel(const mySlam::Vec3& pc);
    mySlam::Vec3 pixel2Camera(const mySlam::Vec2& pp, double depth = 1);

    mySlam::Vec3 pixel2World(const mySlam::Vec2& pp, const SE3& TCW, double depth = 1);
    mySlam::Vec2 world2Pixel(const mySlam::Vec3& pw, const SE3& TCW);
};

#endif  // CAMERA_H