#pragma once

#ifndef VIEWER_H
#define VIEWER_H

#include <pangolin/pangolin.h>
#include "commonInclude.h"
#include "frame.h"
#include "map.h"
#include "thread"

class Viewer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void setMap(Map::Ptr map) {
        _map = map;
    }

    void close();

    void addCurrentFrame(Frame::Ptr _currentFrame);

    void updateMap();

private:
    void threadLoop();
    void drawFrame(Frame::Ptr frame, const float* color);
    void drawMapPoints();
    void followCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    cv::Mat PlotFrameImage();

    Frame::Ptr _currentFrame = nullptr;
    Map::Ptr _map = nullptr;

    std::thread _viewerThread;
    bool _viewerRunning = true;

    std::unordered_map<unsigned long, Frame::Ptr> _activeKeyframes;
    std::unordered_map<unsigned long, MapPoint::Ptr> _activeLandmarks;
    bool _mapUpdated = false;

    std::mutex _viewerDataMutex;
};

#endif  // VIEWER_H