//
// Created by marko on 21.05.19..
//

#ifndef LIBREALSENSE2_CS_FACTORY_H
#define LIBREALSENSE2_CS_FACTORY_H

#include "smcs_cpp/CameraSDK.h"
#include "smcs_cpp/IImageBitmap.h"
#include "context.h"

namespace librealsense {
    typedef enum cs_stream {
        CS_STREAM_COLOR,
        CS_STREAM_DEPTH
    } cs_stream;

    typedef enum cs_camera_model {
        CS_UCC2592C,
        CS_UCC1932C,
        CS_D435E,
        CS_UNDEFINED
    };

    namespace platform {
        class cs_device {
        public:
            explicit cs_device(platform::cs_device_info hwm)
                    : _device_info(std::move(hwm)),
                      _power_state(D3),
                      _is_color_capturing(false),
                      _is_depth_capturing(false),
                      _is_started(false),
                      _color_thread(nullptr),
                      _depth_thread(nullptr),
                      _connected_device(NULL) {
                printf("Stvaram cs device\n");
                _smcs_api = smcs::GetCameraAPI();
                auto devices = _smcs_api->GetAllDevices();

                for (int i = 0; i < devices.size(); i++) {
                    auto serial = devices[i]->GetSerialNumber();
                    if (!serial.compare(_device_info.serial)) {
                        _connected_device = devices[i];

                        if (_connected_device == NULL || !_connected_device->Connect())
                            throw wrong_api_call_sequence_exception("Could not create CS device!");
                    }
                }
            }

            ~cs_device() {
                printf("Ubijam cs device\n");
                if (_connected_device->IsOnNetwork()) _connected_device->Disconnect();
            }

            power_state set_power_state(power_state state);

            void stream_on(std::function<void(const notification &n)> error_handler, cs_stream stream);

            void probe_and_commit(stream_profile profile, frame_callback callback, cs_stream stream);

            void close(stream_profile profile, cs_stream stream);

            void color_image_poll();

            void depth_image_poll();

            power_state get_power_state() const { return _power_state; }

            bool get_pu(rs2_option opt, int32_t &value);

            bool set_pu(rs2_option opt, int32_t value);

            control_range get_pu_range(rs2_option option);

            std::vector <stream_profile> get_profiles();

            bool reset(void);

        protected:
            void prepare_capture_buffers();

            void color_capture_loop();

            void depth_capture_loop();

            void set_format(stream_profile profile);

            std::function<void(const notification &n)> _color_error_handler, _depth_error_handler;
            stream_profile _color_profile, _depth_profile;

        private:
            std::string get_cs_param_name(rs2_option option);

            bool get_cs_param_min(rs2_option option, int32_t &value);

            bool get_cs_param_max(rs2_option option, int32_t &value);

            int32_t get_cs_param_step(rs2_option option);

            bool get_cs_param_value(rs2_option option, int32_t &value);

            bool set_cs_param(rs2_option option, int32_t value);

            //std::vector<std::shared_ptr<buffer>> _buffers;
            std::unique_ptr <std::thread> _color_thread, _depth_thread;
            platform::cs_device_info _device_info;
            power_state _power_state;
            std::atomic<bool> _is_color_capturing, _is_depth_capturing;
            std::atomic<bool> _is_started;
            std::mutex _power_lock, _stream_lock;
            smcs::ICameraAPI _smcs_api;
            smcs::IDevice _connected_device;
            frame_callback _color_callback, _depth_callback;
        };
    }

    class cs_info : public device_info
    {
    public:
        std::shared_ptr<device_interface> create(std::shared_ptr<context> ctx,
                                                 bool register_device_notifications) const override;

        cs_info(std::shared_ptr<context> ctx,
                platform::cs_device_info hwm)
                : device_info(ctx),
                  _hwm(std::move(hwm))
        {}

        static std::vector<std::shared_ptr<device_info>> pick_cs_devices(
                std::shared_ptr<context> ctx,
                std::vector<platform::cs_device_info>& cs);

        platform::backend_device_group get_device_data() const override
        {
            return platform::backend_device_group({ _hwm });
        }

        static std::vector<platform::cs_device_info> query_cs_devices();

        static cs_camera_model get_camera_model(std::string pid);

        static bool is_timestamp_supported(std::string pid);

    private:
        platform::cs_device_info _hwm;
    };
}

#endif //LIBREALSENSE2_CS_FACTORY_H
