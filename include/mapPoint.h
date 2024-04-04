#pragma once

#ifndef MAP_POINT_H
#define MAP_POINT_H

#include "commonInclude.h"

struct Frame;
struct Feature;

struct MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long _id = 0;
    bool _isOutlier = false;
    mySlam::Vec3 _pos = mySlam::Vec3::Zero();  // Position in world
    std::mutex _dataMutex;
    int observedTimes = 0;  // Observed by feature matching
    std::list<std::weak_ptr<Feature>> _observations;

public:
    MapPoint(){};
    MapPoint(long id, mySlam::Vec3 position);

    mySlam::Vec3 getPos();
    void setPos(mySlam::Vec3 pos);

    void addObservation(std::shared_ptr<Feature> feature);

    void removeObservations(std::shared_ptr<Feature> feature);

    std::list<std::weak_ptr<Feature>> getObservations();

    static MapPoint::Ptr createNewMapPoint();
};

#endif  // MAP_POINT_H