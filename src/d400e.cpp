// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "d400e.h"
#include "smcs_cpp/CameraSDK.h"

namespace librealsense
{
    namespace d400e
    {
        bool heartbeat_initialized = false;

        void initialize_heartbeat_time()
        {
            if (!heartbeat_initialized)
                set_heartbeat_time(3);
        }

        void set_heartbeat_time(double time)
        {
            smcs::GetCameraAPI()->SetHeartbeatTime(time);
            heartbeat_initialized = true;
        }

        double get_heartbeat_time()
        {
            if (!heartbeat_initialized)
                initialize_heartbeat_time();
            return smcs::GetCameraAPI()->GetHeartbeatTime();
        }
    }
}