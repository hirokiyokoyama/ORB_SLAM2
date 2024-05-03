#include "py_src/cv.h"
#include "py_src/MapPoint.h"
#include "py_src/System.h"

PYBIND11_MODULE(orbslam2, m) {
    NDArrayConverter::init_numpy();

    m.doc() = "ORB_SLAM2"; // optional module docstring

    bind_cv(m);
    bind_MapPoint(m);
    bind_System(m);
}