#include "frontend.h"
#include "algorithm.h"
#include "backend.h"
#include "config.h"
#include "g2oTypes.h"
#include "viewer.h"

Frontend::Frontend() {
    _numFeaturesInit = Config::Get<int>("numFeaturesInit");
    _numFeatures = Config::Get<int>("numFeatures");
    _gftt = cv::GFTTDetector::create(_numFeatures, 0.01, 20);
}

bool Frontend::addFrame(Frame::Ptr frame) {
    _currentFrame = frame;

    switch (_status) {
    case FrontEndStatus::INITIALIZE:
        stereoInit();
        break;
    case FrontEndStatus::TRACKING_GOOD:
    case FrontEndStatus::TRACKING_BAD:
        Track();
        break;
    case FrontEndStatus::TESTING:
    case FrontEndStatus::TRACKING_LOST:
        Reset();
        break;
    }

    _lastFrame = _currentFrame;
    return true;
}

void Frontend::setMap(Map::Ptr map) {
    _map = map;
}

void Frontend::setBackend(std::shared_ptr<Backend> backend) {
    _backend = backend;
}

void Frontend::setViewer(std::shared_ptr<Viewer> viewer) {
    _viewer = viewer;
}

void Frontend::setCameras(Camera::Ptr left, Camera::Ptr right) {
    _leftCamera = left;
    _rightCamera = right;
}

FrontEndStatus Frontend::getFrontEndStatus() {
    return _status;
}

bool Frontend::Track() {
    if (_lastFrame) {
        _currentFrame->setPose(_relativeMotion * _lastFrame->getPose());
    }

    int numTrackLast = trackLastFrame();
    _trackingInliers = estimateCurrentPose();

    if (_trackingInliers > _numFeaturesTracking) {
        // tracking good
        _status = FrontEndStatus::TRACKING_GOOD;
    } else if (_trackingInliers > _numFeaturesTrackingBad) {
        // tracking bad
        _status = FrontEndStatus::TRACKING_BAD;
    } else {
        _status = FrontEndStatus::TRACKING_LOST;
    }

    insertKeyFrame();
    _relativeMotion = _currentFrame->getPose() * _lastFrame->getPose().inverse();

    if (_viewer) {
        _viewer->addCurrentFrame(_currentFrame);
    }
    return true;
}

bool Frontend::Reset() {
    LOG(INFO) << "Reset tracking";
    // implement retracking code here
    return true;
}

int Frontend::trackLastFrame() {
    std::vector<cv::Point2f> kpsLast, kpsCurrent;
    for (auto& kp : _lastFrame->_featuresLeft) {
        if (kp->_mapPoint.lock()) {
            auto mp = kp->_mapPoint.lock();
            auto px = _leftCamera->world2Pixel(mp->_pos, _currentFrame->getPose());
            kpsLast.emplace_back(kp->_position.pt);
            kpsCurrent.emplace_back(cv::Point2f(px[0], px[1]));
        } else {
            kpsLast.emplace_back(kp->_position.pt);
            kpsCurrent.emplace_back(kp->_position.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;

    cv::calcOpticalFlowPyrLK(_lastFrame->_leftImg, _currentFrame->_leftImg, kpsLast, kpsCurrent, status, error, cv::Size(11, 11), 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

    int numGoodPts = 0;

    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            cv::KeyPoint kp(kpsCurrent[i], 7);
            Feature::Ptr feature(new Feature(_currentFrame, kp));
            feature->_mapPoint = _lastFrame->_featuresLeft[i]->_mapPoint;
            _currentFrame->_featuresLeft.emplace_back(feature);
            numGoodPts++;
        }
    }

    LOG(INFO) << "Found " << numGoodPts << " in the last frame";
    return numGoodPts;
}

int Frontend::estimateCurrentPose() {
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    mySlam::VertexPose* vertex_pose = new mySlam::VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(_currentFrame->getPose());
    optimizer.addVertex(vertex_pose);

    // intrinsics
    mySlam::Mat33 K = _leftCamera->getIntrinsics();

    // edges
    int index = 1;
    std::vector<mySlam::EdgeProjectionPoseOnly*> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < _currentFrame->_featuresLeft.size(); ++i) {
        auto mp = _currentFrame->_featuresLeft[i]->_mapPoint.lock();
        if (mp) {
            features.push_back(_currentFrame->_featuresLeft[i]);
            mySlam::EdgeProjectionPoseOnly* edge =
                new mySlam::EdgeProjectionPoseOnly(mp->_pos, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(_currentFrame->_featuresLeft[i]->_position.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // Estimate the Pose and determine outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(_currentFrame->getPose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->_isOutlier) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->_isOutlier = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->_isOutlier = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    _currentFrame->setPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n"
              << _currentFrame->getPose().matrix();

    for (auto& feat : features) {
        if (feat->_isOutlier) {
            feat->_mapPoint.reset();
            feat->_isOutlier = false;  // can be included in future scope
        }
    }
    return features.size() - cnt_outlier;
}

bool Frontend::insertKeyFrame() {
    if (_trackingInliers >= _numFeaturesNeededForKeyFrame) {  // still have enough features
        return false;
    }

    _currentFrame->setKeyframe();
    _map->insertKeyFrame(_currentFrame);

    LOG(INFO) << "set frame " << _currentFrame->_id << " as keyframe " << _currentFrame->_keyframeID;

    setObservationsForKeyFrame();
    detectFeatures();
    findFeaturesInRight();
    triangulateNewPoints();

    // update backend map
    _backend->updateMap();

    if (_viewer) {
        _viewer->updateMap();
    }

    return true;
}

bool Frontend::stereoInit() {
    int numFeaturesLeft = detectFeatures();
    int numFeaturesRight = findFeaturesInRight();

    if (numFeaturesRight < _numFeaturesInit) {
        return false;
    }

    bool buildMapSuccess = buildInitMap();
    if (buildMapSuccess) {
        _status = FrontEndStatus::TRACKING_GOOD;
        if (_viewer) {
            _viewer->addCurrentFrame(_currentFrame);
            _viewer->updateMap();
        }
        return true;
    }
    return false;
}

int Frontend::detectFeatures() {
    cv::Mat mask(_currentFrame->_leftImg.size(), CV_8UC1, 255);
    for (auto& feat : _currentFrame->_featuresLeft) {
        cv::rectangle(mask, feat->_position.pt - cv::Point2f(10, 10), feat->_position.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    _gftt->detect(_currentFrame->_leftImg, keypoints, mask);
    int cntDetected = 0;
    for (auto& kp : keypoints) {
        _currentFrame->_featuresLeft.emplace_back(Feature::Ptr(new Feature(_currentFrame, kp)));
        cntDetected++;
    }
    LOG(INFO) << "Detected " << cntDetected << " new features";
    return cntDetected;
}

int Frontend::findFeaturesInRight() {
    std::vector<cv::Point2f> kpsLeft, kpsRight;
    for (auto& kp : _currentFrame->_featuresLeft) {
        kpsLeft.emplace_back(kp->_position.pt);
        auto mp = kp->_mapPoint.lock();
        if (mp) {
            auto px = _rightCamera->world2Pixel(mp->_pos, _currentFrame->getPose());
            kpsRight.emplace_back(cv::Point2f(px[0], px[1]));
        } else {
            kpsRight.emplace_back(kp->_position.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;

    cv::calcOpticalFlowPyrLK(_currentFrame->_leftImg, _currentFrame->_rightImg, kpsLeft, kpsRight, status, error, cv::Size(11, 11), 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

    int numGoodPts = 0;

    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            cv::KeyPoint kp(kpsRight[i], 7);
            Feature::Ptr feature(new Feature(_currentFrame, kp));
            feature->_isOnLeftImg = false;
            _currentFrame->_featuresRight.emplace_back(feature);
            numGoodPts++;
        } else {
            _currentFrame->_featuresRight.emplace_back(nullptr);
        }
    }

    LOG(INFO) << "Found " << numGoodPts << " in the right image";
    return numGoodPts;
}

bool Frontend::buildInitMap() {
    std::vector<SE3> poses{_leftCamera->getPose(), _rightCamera->getPose()};
    size_t cntInitLandmarks = 0;
    for (size_t i = 0; i < _currentFrame->_featuresLeft.size(); i++) {
        if (_currentFrame->_featuresRight[i] == nullptr)
            continue;
        std::vector<mySlam::Vec3> points{
            _leftCamera->pixel2Camera(mySlam::Vec2(_currentFrame->_featuresLeft[i]->_position.pt.x, _currentFrame->_featuresLeft[i]->_position.pt.y)),
            _rightCamera->pixel2Camera(mySlam::Vec2(_currentFrame->_featuresRight[i]->_position.pt.x, _currentFrame->_featuresRight[i]->_position.pt.y))};
        mySlam::Vec3 pWorld = mySlam::Vec3::Zero();

        if (triangulation(poses, points, pWorld) && pWorld[2] > 0) {
            auto newMapPoint = MapPoint::createNewMapPoint();
            newMapPoint->setPos(pWorld);
            newMapPoint->addObservation(_currentFrame->_featuresLeft[i]);
            newMapPoint->addObservation(_currentFrame->_featuresRight[i]);
            _currentFrame->_featuresLeft[i]->_mapPoint = newMapPoint;
            _currentFrame->_featuresRight[i]->_mapPoint = newMapPoint;
            _map->insertMapPoint(newMapPoint);
            cntInitLandmarks++;
        }
    }
    _currentFrame->setKeyframe();
    _map->insertKeyFrame(_currentFrame);
    _backend->updateMap();

    LOG(INFO) << "Initial map created with " << cntInitLandmarks << " map points";

    return true;
}

int Frontend::triangulateNewPoints() {
    std::vector<SE3> poses{_leftCamera->getPose(), _rightCamera->getPose()};
    SE3 currentPoseTWC = _currentFrame->getPose().inverse();
    int cntTriangulatedPts = 0;

    for (size_t i = 0; i < _currentFrame->_featuresLeft.size(); i++) {
        if (_currentFrame->_featuresLeft[i]->_mapPoint.expired() && _currentFrame->_featuresRight[i] != nullptr) {
            std::vector<mySlam::Vec3> points{
                _leftCamera->pixel2Camera(mySlam::Vec2(_currentFrame->_featuresLeft[i]->_position.pt.x, _currentFrame->_featuresLeft[i]->_position.pt.y)),
                _rightCamera->pixel2Camera(mySlam::Vec2(_currentFrame->_featuresRight[i]->_position.pt.x, _currentFrame->_featuresRight[i]->_position.pt.y))};
            mySlam::Vec3 pWorld = mySlam::Vec3::Zero();

            if (triangulation(poses, points, pWorld) && pWorld[2] > 0) {
                auto newMapPoint = MapPoint::createNewMapPoint();
                pWorld = currentPoseTWC * pWorld;
                newMapPoint->setPos(pWorld);
                newMapPoint->addObservation(_currentFrame->_featuresLeft[i]);
                newMapPoint->addObservation(_currentFrame->_featuresRight[i]);
                _currentFrame->_featuresLeft[i]->_mapPoint = newMapPoint;
                _currentFrame->_featuresRight[i]->_mapPoint = newMapPoint;
                _map->insertMapPoint(newMapPoint);
                cntTriangulatedPts++;
            }
        }
    }

    LOG(INFO) << "New Landmarks:  " << cntTriangulatedPts;
    return cntTriangulatedPts;
}

void Frontend::setObservationsForKeyFrame() {
    for (auto& feat : _currentFrame->_featuresLeft) {
        auto mp = feat->_mapPoint.lock();
        if (mp) {
            mp->addObservation(feat);
        }
    }
}