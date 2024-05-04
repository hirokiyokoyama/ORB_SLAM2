#pragma once

#include "py_src/common.h"
#include <KeyFrame.h>

class MapPointView;

class KeyFrameView
{
public:
    typedef std::shared_ptr<KeyFrameView> KFPtr;
    typedef std::shared_ptr<MapPointView> MPPtr;

    static KFPtr get_instance(ORB_SLAM2::KeyFrame* kf, bool create=true);

    cv::Mat pose;
    cv::Mat camera_center;
    cv::Mat stereo_center;

    bool bad;

    long unsigned int id;
    long unsigned int frameId;
    double timeStamp;

    float fx, fy, cx, cy, bf, b, thDepth;

    std::vector<cv::KeyPoint> keyPoints;
    std::vector<cv::KeyPoint> undistortedKeyPoints;
    std::vector<float> uRight; // negative value for monocular points
    std::vector<float> depth; // negative value for monocular points
    cv::Mat descriptors;

    std::map<KFPtr,int> connectedKeyFrames;
    KFPtr parent;
    std::set<KFPtr> children;
    std::set<KFPtr> loopEdges;

    std::vector<MPPtr> mapPoints;

protected:
    KeyFrameView() {}
    static std::map<ORB_SLAM2::KeyFrame*, KFPtr> instances;
};

void bind_KeyFrame(pybind11::module & m);
