#include "mapPoint.h"
#include "feature.h"

MapPoint::MapPoint(long id, mySlam::Vec3 position)
    : _id(id), _pos(position) {}

mySlam::Vec3 MapPoint::getPos() {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    return _pos;
}

void MapPoint::setPos(mySlam::Vec3 pos) {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    _pos = pos;
}

void MapPoint::addObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    _observations.emplace_back(feature);
    observedTimes++;
}

void MapPoint::removeObservations(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    for (auto iter = _observations.begin(); iter != _observations.end(); iter++) {
        if (iter->lock() == feature) {
            _observations.erase(iter);
            observedTimes--;
            break;
        }
    }
}

std::list<std::weak_ptr<Feature>> MapPoint::getObservations() {
    std::unique_lock<std::mutex> _lck(_dataMutex);
    return _observations;
}

MapPoint::Ptr MapPoint::createNewMapPoint() {
    static long factoryID = 0;
    MapPoint::Ptr newMapPoint(new MapPoint);
    newMapPoint->_id = factoryID++;
    return newMapPoint;
}