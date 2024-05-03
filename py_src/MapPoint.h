#pragma once

#include "py_src/common.h"
#include <MapPoint.h>

class MapPointView
{
public:
    MapPointView(ORB_SLAM2::MapPoint* mp);

    cv::Mat worldPos;
    cv::Mat normal;
    //std::shared_ptr<KeyFrame> referenceKeyFrame;
    //std::map<std::shared_ptr<KeyFrame>, size_t> observations;
    bool bad;
    float foundRatio;
    int found;
    cv::Mat descriptor;
    float minDistanceInvariance;
    float maxDistanceInvariance;
};

void bind_MapPoint(pybind11::module & m);
