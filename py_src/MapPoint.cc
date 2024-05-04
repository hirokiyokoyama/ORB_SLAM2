#include "py_src/MapPoint.h"
#include "py_src/KeyFrame.h"

std::map<ORB_SLAM2::MapPoint*, MapPointView::MPPtr> MapPointView::instances;

std::shared_ptr<MapPointView> MapPointView::get_instance(ORB_SLAM2::MapPoint* mp, bool create) {
    if(!mp)
        return std::shared_ptr<MapPointView>();

    auto it = instances.find(mp);
    if(it != instances.end())
        return it->second;

    if(!create)
        return std::shared_ptr<MapPointView>();

    MapPointView* mpv = new MapPointView();
    MPPtr ptr = MPPtr(mpv);
    instances[mp] = ptr;

    mpv->worldPos = mp->GetWorldPos();
    mpv->normal = mp->GetNormal();
    mpv->bad = mp->isBad();
    mpv->foundRatio = mp->GetFoundRatio();
    mpv->found = mp->GetFound();
    mpv->descriptor = mp->GetDescriptor();
    mpv->minDistanceInvariance = mp->GetMinDistanceInvariance();
    mpv->maxDistanceInvariance = mp->GetMaxDistanceInvariance();

    mpv->referenceKeyFrame = KeyFrameView::get_instance(mp->GetReferenceKeyFrame());
    std::map<ORB_SLAM2::KeyFrame*, size_t> obs = mp->GetObservations();
    for(auto it=obs.begin(); it != obs.end(); ++it) {
        KFPtr obv = KeyFrameView::get_instance(it->first, true);
        mpv->observations[obv] = it->second;
    }

    return ptr;
}

void bind_MapPoint(py::module& m) {
    py::class_<MapPointView>(m, "MapPoint")
        .def_readonly("world_pos", &MapPointView::worldPos)
        .def_readonly("normal", &MapPointView::normal)
        .def_readonly("bad", &MapPointView::bad)
        .def_readonly("found_ratio", &MapPointView::foundRatio)
        .def_readonly("found", &MapPointView::found)
        .def_readonly("descriptor", &MapPointView::descriptor)
        .def_readonly("min_distance_invariance", &MapPointView::minDistanceInvariance)
        .def_readonly("max_distance_invariance", &MapPointView::maxDistanceInvariance)
        .def_readonly("reference_key_frame", &MapPointView::referenceKeyFrame)
        .def_readonly("observations", &MapPointView::observations);
}