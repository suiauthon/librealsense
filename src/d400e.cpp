// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "d400e.h"
#include "smcs_cpp/CameraSDK.h"

namespace librealsense
{
    namespace d400e
    {
        // class heartbeat_time

        heartbeat_time::heartbeat_time()
        {
            constexpr seconds DEFAULT_HEARTBEAT_TIME = 3;
            set(DEFAULT_HEARTBEAT_TIME);
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
        
        // !class heartbeat_time

        // class device_diagnostics

        device_diagnostics::device_diagnostics()
        {
        }

        device_diagnostics& device_diagnostics::get_instance()
        {
            static device_diagnostics instance;
            return instance;
        }

        int device_diagnostics::set(const rs2_device* device, int toggle)
        {
            int r_status = dev_diag_status_NOK;

            if (device != nullptr) {

                uint64_t dev_serial_u = 0;
                const std::string& dev_serial = device->device->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

                try {
                    dev_serial_u = std::stoll(dev_serial, 0, 16);
                }
                catch (...) {
                    dev_serial_u = 0;
                }

                if (dev_serial_u != 0) {

                    smcs::IDevice cs_device = smcs::GetCameraAPI()->GetDeviceByMac(dev_serial_u);
                    if (cs_device.IsValid()) {

                        if (cs_device->IsConnected()) {

                            bool b_status = false;

                            if (toggle == (int)dev_diag_toggle_ON) {
                                b_status = cs_device->SetStringNodeValue("DebugInformation", "On");
                            }
                            else if (toggle == (int)dev_diag_toggle_OFF) {
                                b_status = cs_device->SetStringNodeValue("DebugInformation", "Off");
                            }

                            if (b_status) {
                                r_status = dev_diag_status_OK;
                            }
                        }
                    }
                }
            }

            return r_status;
        }

        // !class device_diagnostics
    }
}