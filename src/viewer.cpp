#include "viewer.h"
#include "feature.h"
#include "frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

Viewer::Viewer() {
    _viewerThread = std::thread(std::bind(&Viewer::threadLoop, this));
}

void Viewer::close() {
    _viewerRunning = false;
    _viewerThread.join();
}

void Viewer::addCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(_viewerDataMutex);
    _currentFrame = current_frame;
}

void Viewer::updateMap() {
    std::unique_lock<std::mutex> lck(_viewerDataMutex);
    assert(_map != nullptr);
    _activeKeyframes = _map->getActiveKeyFrames();
    _activeLandmarks = _map->getActiveMapPoints();
    _mapUpdated = true;
}

void Viewer::threadLoop() {
    pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState visCamera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View& visDisplay =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(visCamera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() && _viewerRunning) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        visDisplay.Activate(visCamera);

        std::unique_lock<std::mutex> lock(_viewerDataMutex);
        if (_currentFrame) {
            drawFrame(_currentFrame, green);
            followCurrentFrame(visCamera);

            cv::Mat img = PlotFrameImage();
            cv::imshow("image", img);
            cv::waitKey(1);
        }

        if (_map) {
            drawMapPoints();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }

    LOG(INFO) << "Stop viewer";
}

cv::Mat Viewer::PlotFrameImage() {
    cv::Mat imgOut;
    cv::cvtColor(_currentFrame->_leftImg, imgOut, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < _currentFrame->_featuresLeft.size(); ++i) {
        if (_currentFrame->_featuresLeft[i]->_mapPoint.lock()) {
            auto feat = _currentFrame->_featuresLeft[i];
            cv::circle(imgOut, feat->_position.pt, 2, cv::Scalar(0, 250, 0),
                       2);
        }
    }
    return imgOut;
}

void Viewer::followCurrentFrame(pangolin::OpenGlRenderState& visCamera) {
    SE3 Twc = _currentFrame->getPose().inverse();
    pangolin::OpenGlMatrix m = pangolin::OpenGlMatrix::ColMajor4x4(Twc.matrix().data());
    visCamera.Follow(m, true);
}

void Viewer::drawFrame(Frame::Ptr frame, const float* color) {
    SE3 Twc = frame->getPose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::drawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : _activeKeyframes) {
        drawFrame(kf.second, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : _activeLandmarks) {
        auto pos = landmark.second->getPos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}
