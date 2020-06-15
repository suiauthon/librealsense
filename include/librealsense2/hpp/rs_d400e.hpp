// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#ifndef LIBREALSENSE_RS2_D400E_HPP
#define LIBREALSENSE_RS2_D400E_HPP

#include "rs_types.hpp"

namespace rs2
{
    namespace d400e
    {
        /**
        * set heartbeat time for d400e devices
        * \param[in] time     heartbeat time in seconds
        */
        inline void set_heartbeat_time(double time)
        {
            rs2_error* e = nullptr;
            rs2_d400e_set_heartbeat_time(time, &e);
            rs2::error::handle(e);
        }

        /**
        * retrieve heartbeat time for d400e devices
        * \return             heartbeat time in seconds
        */
        inline double get_heartbeat_time()
        {
            rs2_error* e = nullptr;
            auto time = rs2_d400e_get_heartbeat_time(&e);
            rs2::error::handle(e);

            return time;
        }

        /**
        * set buffer count for d400e devices
        * \param[in] buffer_count     number of buffers
        */
        inline void set_buffer_count(int buffer_count)
        {
            rs2_error* e = nullptr;
            rs2_d400e_set_buffer_count(buffer_count, &e);
            rs2::error::handle(e);
        }

        /**
        * retrieve buffer count for d400e devices
        * \return             buffer count in seconds
        */
        inline int get_buffer_count()
        {
            rs2_error* e = nullptr;
            auto buffer_count = rs2_d400e_get_buffer_count(&e);
            rs2::error::handle(e);

            return buffer_count;
        }

        inline rs2_cs_camera_config* create_cs_camera_config()
        {
            rs2_error* e = nullptr;
            auto cam_conf = rs2_d400e_create_cs_camera_config(&e);
            rs2::error::handle(e);

            return cam_conf;
        }

        inline void add_ip_to_cs_camera_config(rs2_cs_camera_config* cs_config, std::string ip)
        {
            rs2_error* e = nullptr;
            const char *cstr = ip.c_str();
            rs2_d400e_add_ip_to_cs_camera_config(cs_config, cstr, &e);
            rs2::error::handle(e);
        }

        inline void add_sn_to_cs_camera_config(rs2_cs_camera_config* cs_config, std::string sn)
        {
            rs2_error* e = nullptr;
            const char *cstr = sn.c_str();
            rs2_d400e_add_sn_to_cs_camera_config(cs_config, cstr, &e);
            rs2::error::handle(e);
        }
    }
}
#endif // LIBREALSENSE_RS2_D400E_HPP
