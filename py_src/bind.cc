#include "py_src/System.h"

PYBIND11_MODULE(orb_slam2, m) {
    NDArrayConverter::init_numpy();

    m.doc() = "ORB_SLAM2"; // optional module docstring

    bind_System(m);
}