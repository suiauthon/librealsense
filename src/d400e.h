// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#pragma once

namespace librealsense
{
    namespace d400e
    {
        void initialize_heartbeat_time();
        void set_heartbeat_time(double time);
        double get_heartbeat_time();
    }
}