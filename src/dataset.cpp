#include "dataset.h"

Dataset::Dataset(const std::string& path)
    : _path(path){};

bool Dataset::init() {
    // read camera intrinsics and extrinsics
    std::ifstream fin(_path + "/calib.txt");
    if (!fin) {
        LOG(ERROR) << "cannot find" << _path << "/calib.txt";
        return false;
    }

    for (int i = 0; i < 4; i++) {
        char cameraName[4];
        for (int j = 0; j < 4; j++) {
            fin >> cameraName[j];
        }

        double projectionData[12];
        for (int j = 0; j < 12; j++) {
            fin >> projectionData[j];
        }

        mySlam::Mat33 K;
        K << projectionData[0], projectionData[1], projectionData[2],
            projectionData[4], projectionData[5], projectionData[6],
            projectionData[8], projectionData[9], projectionData[10];

        mySlam::Vec3 t;
        t << projectionData[3], projectionData[7], projectionData[11];
        t = K.inverse() * t;

        K = K * 0.5;
        Camera::Ptr newCamera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(), SE3(SO3(), t)));
        _cameras.emplace_back(newCamera);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }

    fin.close();
    _currentImgIndex = 0;
    return true;
}

Frame::Ptr Dataset::nextFrame() {
    cv::Mat imgLeft, imgRight;
    boost::format fmt("%s/image_%d/%06d.png");
    imgLeft = cv::imread((fmt % _path % 0 % _currentImgIndex).str(), cv::IMREAD_GRAYSCALE);
    imgRight = cv::imread((fmt % _path % 1 % _currentImgIndex).str(), cv::IMREAD_GRAYSCALE);

    if (imgLeft.data == nullptr || imgRight.data == nullptr) {
        LOG(WARNING) << "Cannot find images at index: " << _currentImgIndex;
        return nullptr;
    }
    cv::Mat imgLeftResized, imgRightResized;
    cv::resize(imgLeft, imgLeftResized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    cv::resize(imgRight, imgRightResized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

    auto newFrame = Frame::createFrame();
    newFrame->_leftImg = imgLeftResized;
    newFrame->_rightImg = imgRightResized;
    _currentImgIndex++;
    return newFrame;
}

Camera::Ptr Dataset::getCamera(int cameraID) const {
    return _cameras.at(cameraID);
}