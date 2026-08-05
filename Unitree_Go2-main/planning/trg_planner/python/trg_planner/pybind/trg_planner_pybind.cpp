/**
 * Copyright 2025, Korea Advanced Institute of Science and Technology
 * Massachusetts Institute of Technology,
 * Daejeon, 34051
 * All Rights Reserved
 * Authors: Dongkyu Lee, et al.
 * See LICENSE for the license information
 */
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "trg_planner/include/planner/trg_planner.h"

namespace py = pybind11;

PYBIND11_MODULE(trg_planner, m) {
  m.doc()               = "Pybind11 bindings for TRG-planner library";
  m.attr("__version__") = "1.0.0";

  py::class_<TRG, std::shared_ptr<TRG>>(m, "TRG")
      .def(py::init<bool, float, float, int, float, float, float, float, float>())
      .def("getGraphCopy", &TRG::getGraphCopy, "Get the graph", py::arg("type"));

  py::class_<TRG::Edge>(m, "Edge")
      .def(py::init<int, float, float>())
      .def_readwrite("dst_id", &TRG::Edge::dst_id_)
      .def_readwrite("weight", &TRG::Edge::weight_)
      .def_readwrite("dist", &TRG::Edge::dist_);

  py::enum_<TRG::NodeState>(m, "NodeState")
      .value("Valid", TRG::NodeState::Valid)
      .value("Invalid", TRG::NodeState::Invalid)
      .value("Frontier", TRG::NodeState::Frontier);

  py::class_<TRG::Node>(m, "Node")
      .def(py::init<int, Eigen::Vector2f&, float, TRG::NodeState>())
      .def_readwrite("id", &TRG::Node::id_)
      .def_readwrite("pos", &TRG::Node::pos_)
      .def_readwrite("state", &TRG::Node::state_)
      .def_readwrite("edges", &TRG::Node::edges_);

  py::class_<TRGPlanner, std::shared_ptr<TRGPlanner>>(m, "TRGPlanner")
      .def(py::init<>())
      .def("init", &TRGPlanner::init)
      .def("setParams", &TRGPlanner::setParams, "Set parameters from config file")

      // TRG functions
      .def("getTRG", &TRGPlanner::getTRG)

      .def("setPose",
           &TRGPlanner::setPose,
           py::arg("pose")     = Eigen::Vector3f::Zero(),
           py::arg("quat")     = Eigen::Vector4f(1, 0, 0, 0),
           py::arg("frame_id") = "map")
      .def("setObs", &TRGPlanner::setObs, py::arg("obs"))
      .def("setGoal",
           &TRGPlanner::setGoal,
           py::arg("pose") = Eigen::Vector3f::Zero(),
           py::arg("quat") = Eigen::Vector4f(1, 0, 0, 0))

      .def("getPlannedPath", &TRGPlanner::getPlannedPath, py::arg("type") = "smooth")
      .def("getPathInfo", &TRGPlanner::getPathInfo)

      .def("getMapEigen", &TRGPlanner::getMapEigen, py::arg("type") = "preMap")

      .def("getGoalPose", &TRGPlanner::getGoalPose)
      .def("getGoalQuat", &TRGPlanner::getGoalQuat)

      .def("shutdown", &TRGPlanner::shutdown)
      .def("setFlagPathFound", &TRGPlanner::setFlagPathFound, py::arg("flag"))
      .def("getFlagPreMap", &TRGPlanner::getFlagPreMap)
      .def("getFlagPathFound", &TRGPlanner::getFlagPathFound)
      .def("getFlagGoalIn", &TRGPlanner::getFlagGoalIn)
      .def("getFlagGraphInit", &TRGPlanner::getFlagGraphInit);
}
