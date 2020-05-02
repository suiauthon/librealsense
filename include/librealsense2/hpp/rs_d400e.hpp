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
    }
}
#endif // LIBREALSENSE_RS2_D400E_HPP
