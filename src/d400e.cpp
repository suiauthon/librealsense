// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "d400e.h"
#include "smcs_cpp/CameraSDK.h"

namespace librealsense
{
    namespace d400e
    {
        void set_heartbeat_time(double time)
        {
            smcs::GetCameraAPI()->SetHeartbeatTime(time);
        }

        double get_heartbeat_time()
        {
            return smcs::GetCameraAPI()->GetHeartbeatTime();
        }
    }
}