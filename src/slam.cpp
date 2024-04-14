#include "slam.h"
#include "config.h"

/*
TODO:
    1) SLAM::Initialize() ->  set backend map, cameras + set viewer map
    2) SLAM::Run() -> // stop backend and close viewer
*/

SLAM::SLAM(std::string& configPath)
    : _configFilePath(configPath) {}

bool SLAM::Initialize() {
    // read config file
    if (Config::setParameterFile(_configFilePath) == false)
        return false;

    _dataset = Dataset::Ptr(new Dataset(Config::Get<std::string>("datasetDir")));
    if (_dataset->init() != true) {
        LOG(ERROR) << "cannot open dataset";
    }

    // Create components
    // _frontend = Frontend::Ptr(new Frontend);
    // _backend = Backend::Ptr(new Backend);
    // _map = Map::Ptr(new Map);
    // _viewer = Viewer::Ptr(new Viewer);

    // _frontend->setBackend(_backend);
    // _frontend->setMap(_map);
    // _frontend->setViewer(_viewer);
    // _frontend->setCameras(_dataset->getCamera(0), _dataset->getCamera(1));

    // set backend map, cameras

    // set viewer map

    return true;
}

void SLAM::Run() {
    while (true) {
        LOG(INFO) << "SLAM is Running";
        if (Step() == false) {
            break;
        }
    }

    // stop backend and close viewer

    LOG(INFO) << "SLAM stopped!";
}

bool SLAM::Step() {
    Frame::Ptr newFrame = _dataset->nextFrame();
    if (newFrame == nullptr) {
        return false;
    }

    auto t1 = std::chrono::steady_clock::now();
    bool success = _frontend->addFrame(newFrame);
    auto t2 = std::chrono::steady_clock::now();
    auto timeTaken = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    LOG(INFO) << "SLAM time taken: " << timeTaken.count() << " seconds";

    return success;
}

FrontEndStatus SLAM::getFrontEndStatus() {
    return _frontend->getFrontEndStatus();
}