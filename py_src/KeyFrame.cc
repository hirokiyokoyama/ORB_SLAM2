#include "py_src/KeyFrame.h"
#include "py_src/MapPoint.h"

std::map<ORB_SLAM2::KeyFrame*, KeyFrameView::KFPtr> KeyFrameView::instances;

std::shared_ptr<KeyFrameView> KeyFrameView::get_instance(ORB_SLAM2::KeyFrame* kf, bool create) {
    if(!kf)
        return std::shared_ptr<KeyFrameView>();

    auto it = instances.find(kf);
    if(it != instances.end())
        return it->second;

    if(!create)
        return std::shared_ptr<KeyFrameView>();

    KeyFrameView* kfv = new KeyFrameView();
    KFPtr ptr = KFPtr(kfv);
    instances[kf] = ptr;

    kfv->pose = kf->GetPose();
    kfv->camera_center = kf->GetCameraCenter();
    kfv->stereo_center = kf->GetStereoCenter();
    kfv->bad = kf->isBad();
    kfv->id = kf->mnId;
    kfv->frameId = kf->mnFrameId;
    kfv->timeStamp = kf->mTimeStamp;
    kfv->fx = kf->fx;
    kfv->fy = kf->fy;
    kfv->cx = kf->cx;
    kfv->cy = kf->cy;
    kfv->bf = kf->mbf;
    kfv->b = kf->mb;
    kfv->thDepth = kf->mThDepth;
    kfv->keyPoints = kf->mvKeys;
    kfv->undistortedKeyPoints = kf->mvKeysUn;
    kfv->uRight = kf->mvuRight;
    kfv->depth = kf->mvDepth;
    kfv->descriptors = kf->mDescriptors;

    std::set<ORB_SLAM2::KeyFrame*> nkfs = kf->GetConnectedKeyFrames();
    for(auto it=nkfs.begin(); it != nkfs.end(); ++it) {
        int w = kf->GetWeight(*it);
        KFPtr nkfv = get_instance(*it, true);
        kfv->connectedKeyFrames[nkfv] = w;
    }
    kfv->parent = get_instance(kf->GetParent(), true);
    std::set<ORB_SLAM2::KeyFrame*> children = kf->GetChilds();
    for(auto it=children.begin(); it != children.end(); ++it) {
        KFPtr childv = get_instance(*it, true);
        kfv->children.insert(childv);
    }
    std::set<ORB_SLAM2::KeyFrame*> edges = kf->GetLoopEdges();
    for(auto it=edges.begin(); it != edges.end(); ++it) {
        KFPtr edgev = get_instance(*it, true);
        kfv->loopEdges.insert(edgev);
    }
    std::vector<ORB_SLAM2::MapPoint*> mps = kf->GetMapPointMatches();
    for(auto it=mps.begin(); it != mps.end(); ++it) {
        MPPtr mpv = MapPointView::get_instance(*it, true);
        kfv->mapPoints.push_back(mpv);
    }

    return ptr;
}

void bind_KeyFrame(py::module& m) {
    py::class_<KeyFrameView>(m, "KeyFrame")
        .def_readonly("pose", &KeyFrameView::pose)
        .def_readonly("camera_center", &KeyFrameView::camera_center)
        .def_readonly("stereo_center", &KeyFrameView::stereo_center)
        .def_readonly("bad", &KeyFrameView::bad)
        .def_readonly("id", &KeyFrameView::id)
        .def_readonly("frame_id", &KeyFrameView::frameId)
        .def_readonly("time_stamp", &KeyFrameView::timeStamp)
        .def_readonly("fx", &KeyFrameView::fx)
        .def_readonly("fy", &KeyFrameView::fy)
        .def_readonly("cx", &KeyFrameView::cx)
        .def_readonly("cy", &KeyFrameView::cy)
        .def_readonly("bf", &KeyFrameView::bf)
        .def_readonly("b", &KeyFrameView::b)
        .def_readonly("th_depth", &KeyFrameView::thDepth)
        .def_readonly("key_points", &KeyFrameView::keyPoints)
        .def_readonly("undistorted_key_points", &KeyFrameView::undistortedKeyPoints)
        .def_readonly("u_right", &KeyFrameView::uRight)
        .def_readonly("depth", &KeyFrameView::depth)
        .def_readonly("descriptors", &KeyFrameView::descriptors)
        .def_readonly("connected_key_frames", &KeyFrameView::connectedKeyFrames)
        .def_readonly("parent", &KeyFrameView::parent)
        .def_readonly("children", &KeyFrameView::children)
        .def_readonly("loop_edges", &KeyFrameView::loopEdges)
        .def_readonly("map_points", &KeyFrameView::mapPoints);
}