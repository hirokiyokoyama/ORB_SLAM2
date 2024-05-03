#include "py_src/MapPoint.h"

MapPointView::MapPointView(ORB_SLAM2::MapPoint* mp)
: worldPos(mp->GetWorldPos()), normal(mp->GetNormal()), bad(mp->isBad()),
  foundRatio(mp->GetFoundRatio()), found(mp->GetFound()), descriptor(mp->GetDescriptor()),
  minDistanceInvariance(mp->GetMinDistanceInvariance()), maxDistanceInvariance(mp->GetMaxDistanceInvariance()) {
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
        .def_readonly("max_distance_invariance", &MapPointView::maxDistanceInvariance);
}