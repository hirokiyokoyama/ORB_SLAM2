#pragma once

#include "py_src/common.h"
#include <MapPoint.h>

class KeyFrameView;

class MapPointView
{
public:
    typedef std::shared_ptr<KeyFrameView> KFPtr;
    typedef std::shared_ptr<MapPointView> MPPtr;

    static MPPtr get_instance(ORB_SLAM2::MapPoint* mp, bool create=true);

    cv::Mat worldPos;
    cv::Mat normal;
    bool bad;
    float foundRatio;
    int found;
    cv::Mat descriptor;
    float minDistanceInvariance;
    float maxDistanceInvariance;

    KFPtr referenceKeyFrame;
    std::map<KFPtr, size_t> observations;

protected:
    MapPointView() {}
    static std::map<ORB_SLAM2::MapPoint*, MPPtr> instances;
};

void bind_MapPoint(pybind11::module & m);
