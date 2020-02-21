// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "python.hpp"
#include "../include/librealsense2/hpp/rs_d400e.hpp"

void init_d400e(py::module& m) {
    /* rs2_d400e.hpp */
    auto d400e_submodule = m.def_submodule("d400e", "Functionality specific for D400e series devices");
    d400e_submodule.def("set_heartbeat_time", &rs2::d400e::set_heartbeat_time, "set heartbeat time for d400e devices");
    d400e_submodule.def("get_heartbeat_time", &rs2::d400e::get_heartbeat_time, "retrieve heartbeat time for d400e devices");
    /** end rs_d400e.hpp **/
}
