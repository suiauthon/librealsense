// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "python.hpp"
#include "../include/librealsense2/hpp/rs_d400e.hpp"

void init_d400e(py::module& m) {
    /* rs2_d400e.hpp */
    auto d400e_submodule = m.def_submodule("d400e", "Functionality specific for D400e series devices");
    d400e_submodule.def("set_heartbeat_time", &rs2::d400e::set_heartbeat_time, "set heartbeat time for d400e devices");
    d400e_submodule.def("get_heartbeat_time", &rs2::d400e::get_heartbeat_time, "retrieve heartbeat time for d400e devices");
    d400e_submodule.def("set_buffer_count", &rs2::d400e::set_buffer_count, "set number of buffers for d400e devices");
    d400e_submodule.def("get_buffer_count", &rs2::d400e::get_buffer_count, "get number of buffers for d400e devices");
    d400e_submodule.def("create_cs_camera_config", &rs2::d400e::create_cs_camera_config, "create config structure for d400e camera ");
    d400e_submodule.def("add_ip_to_cs_camera_config", &rs2::d400e::add_ip_to_cs_camera_config, "add desired camera ip address to d400e camera config");
    d400e_submodule.def("add_sn_to_cs_camera_config", &rs2::d400e::add_sn_to_cs_camera_config, "add desired camera serial number to d400e camera config");
    /** end rs_d400e.hpp **/
}
