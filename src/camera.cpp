#include "camera.h"

Camera::Camera(){};

Camera::Camera(double fx, double fy, double cx, double cy, double baseline, const SE3& pose)
    : _fx(fx), _fy(fy), _cx(cx), _cy(cy), _baseline(baseline), _pose(pose) {
    _poseInv = _pose.inverse();
};

SE3 Camera::getPose() const {
    return _pose;
}
mySlam::Mat33 Camera::getIntrinsics() const {
    mySlam::Mat33 K;
    K << _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1;
    return K;
}

mySlam::Vec3 Camera::world2Camera(const mySlam::Vec3& pw, const SE3& TCW) {
    return _pose * TCW * pw;
}
mySlam::Vec3 Camera::camera2World(const mySlam::Vec3& pc, const SE3& TCW) {
    return TCW.inverse() * _poseInv* pc;
}

mySlam::Vec2 Camera::camera2Pixel(const mySlam::Vec3& pc) {
    return mySlam::Vec2(
        _fx * pc(0, 0) / pc(2, 0) + _cx,
        _fy * pc(1, 0) / pc(2, 0) * _cy);
}

mySlam::Vec3 Camera::pixel2Camera(const mySlam::Vec2& pp, double depth) {
    return mySlam::Vec3(
        (pp(0, 0) - _cx) * depth / _fx,
        (pp(1, 0) - _cy) * depth / _fy,
        depth);
}

mySlam::Vec3 Camera::pixel2World(const mySlam::Vec2& pp, const SE3& TCW, double depth) {
    return camera2World(pixel2Camera(pp, depth), TCW);
}

mySlam::Vec2 Camera::world2Pixel(const mySlam::Vec3& pw, const SE3& TCW) {
    return camera2Pixel(world2Camera(pw, TCW));
}