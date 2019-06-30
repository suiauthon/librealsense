//
// Created by marko on 21.05.19..
//

#ifndef LIBREALSENSE2_CS_FACTORY_H
#define LIBREALSENSE2_CS_FACTORY_H

#include "smcs_cpp/CameraSDK.h"
#include "smcs_cpp/IImageBitmap.h"
#include "context.h"
#include "cs-option.h"

namespace librealsense {
    typedef enum cs_stream {
        CS_STREAM_DEPTH,
        CS_STREAM_COLOR,
        CS_STREAM_MONO

    } cs_stream;

    typedef enum cs_stream_id {
        CS_STREAM_ID_DEPTH = 0,
        CS_STREAM_ID_COLOR = 1,
        CS_STREAM_ID_MONO = 0

    } cs_stream_id;

    typedef enum cs_camera_model {
        CS_UCC2592C,
        CS_UCC1932C,
        CS_D435E,
        CS_UNDEFINED
    };

    cs_stream_id cs_stream_to_id(cs_stream stream);

    namespace platform {
        class cs_device {
        public:
            explicit cs_device(platform::cs_device_info hwm)
                    : _device_info(std::move(hwm)),
                      _power_state(D3),
                      _is_acquisition_active(0),
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
                        else
                        {
                            INT64 int64Value;
                            if (_connected_device->GetIntegerNodeValue("SourceControlCount", int64Value))
                            {
                                printf("Broj streamoa %d\n", int64Value);
                                _number_of_streams = int64Value;
                                _threads = std::vector<std::unique_ptr <std::thread>>(_number_of_streams);
                                _is_capturing = std::vector<std::atomic<bool>>(_number_of_streams);
                                _callbacks = std::vector<frame_callback>(_number_of_streams);
                                _error_handler = std::vector<std::function<void(const notification &n)>>(_number_of_streams);
                                _profiles = std::vector<stream_profile>(_number_of_streams);

                                for (int i = 0; i < _number_of_streams; i++)
                                {
                                    _threads[i] = nullptr;
                                    _is_capturing[i] = 0;
                                    _callbacks[i] = nullptr;
                                }
                            }
                            else
                                throw wrong_api_call_sequence_exception("Could not create CS device!");
                        }
                    }
                }
            }

            ~cs_device() {
                printf("Ubijam cs device\n");
                if (_connected_device->IsOnNetwork()) _connected_device->Disconnect();
            }

            power_state set_power_state(power_state state);

            void stream_on(std::function<void(const notification &n)> error_handler, cs_stream_id stream);

            void probe_and_commit(stream_profile profile, frame_callback callback, cs_stream_id stream);

            void close(stream_profile profile, cs_stream_id stream);

            void image_poll(cs_stream_id stream);

            power_state get_power_state() const { return _power_state; }

            bool get_pu(rs2_option opt, int32_t &value, cs_stream stream);

            bool set_pu(rs2_option opt, int32_t value, cs_stream stream);

            bool get_auto_exposure_roi(region_of_interest roi, cs_stream stream);
            bool set_auto_exposure_roi(const region_of_interest &roi, cs_stream stream);

            control_range get_pu_range(rs2_option option, cs_stream stream);

            std::vector <stream_profile> get_profiles();

            bool reset(void);

        protected:
            void prepare_capture_buffers();

            void capture_loop(cs_stream_id stream);

            void set_format(stream_profile profile);

            std::vector<std::function<void(const notification &n)>> _error_handler;
            std::vector<stream_profile> _profiles;

        private:
            std::string get_cs_param_name(rs2_option option, cs_stream stream);

            bool get_cs_param_min(rs2_option option, int32_t &value, cs_stream stream);

            bool get_cs_param_max(rs2_option option, int32_t &value, cs_stream stream);

            int32_t get_cs_param_step(rs2_option option, cs_stream stream);

            bool get_cs_param_value(rs2_option option, int32_t &value, cs_stream stream);

            bool set_cs_param(rs2_option option, int32_t value, cs_stream stream);

            void start_acquisition();

            void stop__acquisition();

            uint32_t cs_pixel_format_to_native_pixel_format(std::string cs_format);

            //std::vector<std::shared_ptr<buffer>> _buffers;
            std::vector<std::unique_ptr <std::thread>> _threads;
            platform::cs_device_info _device_info;
            power_state _power_state;
            std::vector<std::atomic<bool>> _is_capturing;
            std::mutex _power_lock, _stream_lock;
            uint8_t _is_acquisition_active, _number_of_streams;
            smcs::ICameraAPI _smcs_api;
            smcs::IDevice _connected_device;
            std::vector<frame_callback> _callbacks;
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
