// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "d400e.h"
#include "smcs_cpp/CameraSDK.h"

namespace librealsense
{
    namespace d400e
    {
        heartbeat_time::heartbeat_time()
        {
            constexpr seconds default_heartbeat_time = 3;
            set(default_heartbeat_time);
        }

        heartbeat_time& heartbeat_time::get_instance()
        {
            static heartbeat_time instance;
            return instance;
        }

        void heartbeat_time::set(seconds time)
        {
            smcs::GetCameraAPI()->SetHeartbeatTime(time);
        }

        seconds heartbeat_time::get()
        {
            return smcs::GetCameraAPI()->GetHeartbeatTime();
        }
    }
}