#include "slam.h"
#include "config.h"

SLAM::SLAM(std::string& configPath)
    : _configFilePath(configPath) {}

bool SLAM::Initialize() {
    // read config file
    if (Config::setParameterFile(_configFilePath) == false)
        return false;

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
    // get frame from dataset, add new frame to front end, if cant add frame -> return false

    auto t1 = std::chrono::steady_clock::now();
    bool success = false;  // add frame here into frontend
    auto t2 = std::chrono::steady_clock::now();
    auto timeTaken = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    LOG(INFO) << "SLAM time taken: " << timeTaken.count() << " seconds";

    return success;
}

FrontEndStatus getFrontEndStatus() {
    // return front end->status
    return FrontEndStatus::TESTING;
}