#include "py_src/System.h"

PYBIND11_MODULE(orbslam2, m) {
    NDArrayConverter::init_numpy();

    m.doc() = "ORB_SLAM2"; // optional module docstring

    bind_System(m);
}