#include "py_src/System.h"
#include "py_src/MapPoint.h"
#include "py_src/KeyFrame.h"

using namespace ORB_SLAM2;

template <typename T, typename V>
std::vector<std::shared_ptr<V> > convert_all(std::vector<T*> src) {
    std::vector<std::shared_ptr<V> > dst;
    dst.reserve(src.size());
    for(typename vector<T*>::iterator it=src.begin(); it!=src.end(); ++it) {
        dst.push_back(V::get_instance(*it, true));
    }
    return dst;
}

void bind_System(py::module& m) {
    py::class_<System> system(m, "System");

    system.def(py::init<const string &, const string &, System::eSensor>())
        .def("track_stereo", &System::TrackStereo)
        .def("track_rgbd", &System::TrackRGBD)
        .def("track_monocular", &System::TrackMonocular)
        .def("activate_localization_mode", &System::ActivateLocalizationMode)
        .def("deactivate_localization_mode", &System::DeactivateLocalizationMode)
        .def("map_changed", &System::MapChanged)
        .def("reset", &System::Reset)
        .def("shutdown", &System::Shutdown)
        .def("save_trajectoryTUM", &System::SaveTrajectoryTUM)
        .def("save_key_frame_trajectoryTUM", &System::SaveKeyFrameTrajectoryTUM)
        .def("save_trajectoryKITTI", &System::SaveTrajectoryKITTI)
        .def("get_tracking_state", &System::GetTrackingState)
        .def("get_tracked_map_points", [](System& sys) {
            return convert_all<MapPoint,MapPointView>(sys.GetTrackedMapPoints());
        })
        .def("get_tracked_key_points", &System::GetTrackedKeyPointsUn)
        .def("get_all_map_points", [](System& sys) {
            return convert_all<MapPoint,MapPointView>(sys.GetAllMapPoints());
        })
        .def("get_all_key_frames", [](System& sys) {
            return convert_all<KeyFrame,KeyFrameView>(sys.GetAllKeyFrames());
        });

    py::enum_<System::eSensor>(system, "Sensor")
        .value("MONOCULAR", System::eSensor::MONOCULAR)
        .value("STEREO", System::eSensor::STEREO)
        .value("RGBD", System::eSensor::RGBD)
        .export_values();
}