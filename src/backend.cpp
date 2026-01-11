#include "backend.h"
#include "algorithm.h"
#include "commonInclude.h"
#include "feature.h"
#include "g2oTypes.h"
#include "map.h"
#include "mapPoint.h"

Backend::Backend() {
    _backendRunning.store(true);
    _backendThread = std::thread(std::bind(&Backend::backendLoop, this));
}

void Backend::updateMap() {
    std::unique_lock<std::mutex> lock(_dataMutex);
    _mapUpdate.notify_one();
}

void Backend::stop() {
    _backendRunning.store(false);
    _mapUpdate.notify_one();
    _backendThread.join();
}

void Backend::backendLoop() {
    while (_backendRunning.load()) {
        std::unique_lock<std::mutex> lock(_dataMutex);
        _mapUpdate.wait(lock);

        Map::KeyFramesType activeKFS = _map->getActiveKeyFrames();
        Map::LandmarksType activeLandmarks = _map->getActiveMapPoints();
        optimize(activeKFS, activeLandmarks);
    }
}

void Backend::optimize(Map::KeyFramesType& keyframes,
                       Map::LandmarksType& landmarks) {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    std::map<unsigned long, mySlam::VertexPose*> vertices;
    unsigned long maxKFID = 0;
    for (auto& keyframe : keyframes) {
        auto kf = keyframe.second;
        mySlam::VertexPose* vertexPose = new mySlam::VertexPose();  // camera vertex_pose
        vertexPose->setId(kf->_keyframeID);
        vertexPose->setEstimate(kf->getPose());
        optimizer.addVertex(vertexPose);
        if (kf->_keyframeID > maxKFID) {
            maxKFID = kf->_keyframeID;
        }

        vertices.insert({kf->_keyframeID, vertexPose});
    }

    std::map<unsigned long, mySlam::VertexXYZ*> verticesLandmarks;

    mySlam::Mat33 K = _leftCamera->getIntrinsics();
    SE3 left_ext = _leftCamera->getPose();
    SE3 right_ext = _rightCamera->getPose();

    // edges
    int index = 1;
    double chi2Th = 5.991;  // robust kernel
    std::map<mySlam::EdgeProjection*, Feature::Ptr> edges_and_features;

    for (auto& landmark : landmarks) {
        if (landmark.second->_isOutlier)
            continue;
        unsigned long landmarkID = landmark.second->_id;
        auto observations = landmark.second->getObservations();
        for (auto& obs : observations) {
            if (obs.lock() == nullptr)
                continue;
            auto feat = obs.lock();
            if (feat->_isOutlier || feat->_frame.lock() == nullptr)
                continue;

            auto frame = feat->_frame.lock();
            mySlam::EdgeProjection* edge = nullptr;
            if (feat->_isOnLeftImg) {
                edge = new mySlam::EdgeProjection(K, left_ext);
            } else {
                edge = new mySlam::EdgeProjection(K, right_ext);
            }

            if (verticesLandmarks.find(landmarkID) ==
                verticesLandmarks.end()) {
                mySlam::VertexXYZ* v = new mySlam::VertexXYZ;
                v->setEstimate(landmark.second->getPos());
                v->setId(landmarkID + maxKFID + 1);
                v->setMarginalized(true);
                verticesLandmarks.insert({landmarkID, v});
                optimizer.addVertex(v);
            }

            if (vertices.find(frame->_keyframeID) !=
                    vertices.end() &&
                verticesLandmarks.find(landmarkID) !=
                    verticesLandmarks.end()) {
                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->_keyframeID));   // pose
                edge->setVertex(1, verticesLandmarks.at(landmarkID));  // landmark
                edge->setMeasurement(toVec2(feat->_position.pt));
                edge->setInformation(mySlam::Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2Th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
                index++;

            } else
                delete edge;
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cntOutlier = 0, cntInlier = 0;
    int iteration = 0;
    while (iteration < 5) {
        cntOutlier = 0;
        cntInlier = 0;
        for (auto& ef : edges_and_features) {
            if (ef.first->chi2() > chi2Th) {
                cntOutlier++;
            } else {
                cntInlier++;
            }
        }
        double inlierRatio = cntInlier / double(cntInlier + cntOutlier);
        if (inlierRatio > 0.5) {
            break;
        } else {
            chi2Th *= 2;
            iteration++;
        }
    }

    for (auto& ef : edges_and_features) {
        if (ef.first->chi2() > chi2Th) {
            ef.second->_isOutlier = true;
            // remove the observation
            ef.second->_mapPoint.lock()->removeObservations(ef.second);
        } else {
            ef.second->_isOutlier = false;
        }
    }
}
