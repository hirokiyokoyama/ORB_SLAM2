#include "py_src/cv.h"

void bind_cv(py::module& m) {
    py::class_<cv::Point2f>(m, "Point2f")
        .def(py::init<>())
        .def_readwrite("x", &cv::Point2f::x)
        .def_readwrite("y", &cv::Point2f::y);

    py::class_<cv::Point3f>(m, "Point3f")
        .def(py::init<>())
        .def_readwrite("x", &cv::Point3f::x)
        .def_readwrite("y", &cv::Point3f::y)
        .def_readwrite("z", &cv::Point3f::z);

    py::class_<cv::KeyPoint>(m, "KeyPoint")
        .def(py::init<>())
        .def(py::init<float, float, float, float, float, int, int>())
        .def_readwrite("pt", &cv::KeyPoint::pt)
        .def_readwrite("size", &cv::KeyPoint::size)
        .def_readwrite("angle", &cv::KeyPoint::angle)
        .def_readwrite("response", &cv::KeyPoint::response)
        .def_readwrite("octave", &cv::KeyPoint::octave)
        .def_readwrite("class_id", &cv::KeyPoint::class_id);
}