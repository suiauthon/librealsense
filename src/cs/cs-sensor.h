//
// Created by marko on 21.05.19..
//

#ifndef LIBREALSENSE2_CS_SENSOR_H
#define LIBREALSENSE2_CS_SENSOR_H

#include <smcs_cpp/CameraSDK.h>
#include <smcs_cpp/IImageBitmap.h>
#include "sensor.h"
#include "types.h"

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

    class cs_firmware_version
    {
    public:
        explicit cs_firmware_version(UINT32 major = 0, UINT32 minor = 0, UINT32 patch = 0, UINT32 build = 0)
            : _major(major)
            , _minor(minor)
            , _patch(patch)
            , _build(build)
        {}

        explicit cs_firmware_version(smcs::IDevice &device);

        bool operator > (const cs_firmware_version &other)
        {
            if (_major > other._major)
                return true;
            else if (_major < other._major)
                return false;

            if (_minor > other._minor)
                return true;
            else if (_minor < other._minor)
                return false;

            if (_patch > other._patch)
                return true;
            else if (_patch < other._patch)
                return false;

            if (_build > other._build)
                return true;
            else if (_build < other._build)
                return false;

            return false;
        }

        bool operator < (const cs_firmware_version &other)
        {
            return !(*this >= other);
        }

        bool operator == (const cs_firmware_version &other)
        {
            return
                (_major == other._major) &&
                (_minor == other._minor) &&
                (_patch == other._patch) &&
                (_build == other._build);
        }

        bool operator >= (const cs_firmware_version &other)
        {
            return (*this > other) || (*this == other);
        }

        bool operator <= (const cs_firmware_version &other)
        {
            return (*this < other) || (*this == other);
        }

    private:
        UINT32 _major, _minor, _patch, _build;
    };

    namespace platform {
        class cs_device {
        public:
            explicit cs_device(platform::cs_device_info hwm)
                    : _device_info(std::move(hwm)),
                      _power_state(D3),
                      _connected_device(NULL) {
                //printf("Stvaram cs device\n");
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
                                //printf("Broj streamoa %d\n", int64Value);
                                _number_of_streams = int64Value;
                                _threads = std::vector<std::unique_ptr <std::thread>>(_number_of_streams);
                                _is_capturing = std::vector<std::atomic<bool>>(_number_of_streams);
                                _callbacks = std::vector<frame_callback>(_number_of_streams);
                                _error_handler = std::vector<std::function<void(const notification &n)>>(_number_of_streams);
                                _profiles = std::vector<stream_profile>(_number_of_streams);
                                _device_version = _connected_device->GetDeviceVersion();
                                _cs_firmware_version = cs_firmware_version(_connected_device);

                                for (int i = 0; i < _number_of_streams; i++)
                                {
                                    select_channel((cs_stream_id)i);

                                    // optimal settings for D435e
                                    _connected_device->SetIntegerNodeValue("GevSCPSPacketSize", 1500);
                                    _connected_device->SetIntegerNodeValue("GevSCPD", 10);

                                    _threads[i] = nullptr;
                                    _is_capturing[i] = false;
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
                //printf("Ubijam cs device\n");
                //This causes only one camera to stream when using pipeline API with multiple devices (rs-multicam example)
                //if (_connected_device->IsOnNetwork()) _connected_device->Disconnect();

                stop_stream(CS_STREAM_ID_DEPTH);
                stop_stream(CS_STREAM_ID_COLOR);
            }

            power_state set_power_state(power_state state);

            void stream_on(std::function<void(const notification &n)> error_handler, cs_stream_id stream);

            void probe_and_commit(stream_profile profile, frame_callback callback, cs_stream_id stream);

            void close(stream_profile profile, cs_stream_id stream);

            void image_poll(cs_stream_id stream);

            power_state get_power_state() const { return _power_state; }

            bool get_pu(rs2_option opt, int32_t &value, cs_stream stream);

            bool set_pu(rs2_option opt, int32_t value, cs_stream stream);

            control_range get_pu_range(rs2_option option, cs_stream stream);

            enum rs2_format get_rgb_format();

            std::vector <stream_profile> get_profiles(cs_stream_id stream);

            bool reset(void);

            std::vector<byte> send_hwm(std::vector<byte>& buffer);

            std::string get_device_version();

        protected:
            void capture_loop(cs_stream_id stream);

            void set_format(stream_profile profile, cs_stream_id stream);

            std::vector<std::function<void(const notification &n)>> _error_handler;
            std::vector<stream_profile> _profiles;

        private:

            std::string get_cs_param_name(rs2_option option, cs_stream stream);

            bool get_cs_param_min(rs2_option option, int32_t &value, cs_stream stream);

            bool get_cs_param_max(rs2_option option, int32_t &value, cs_stream stream);

            int32_t get_cs_param_step(rs2_option option, cs_stream stream);

            bool get_cs_param_value(rs2_option option, int32_t &value, cs_stream stream);

            bool set_cs_param(rs2_option option, int32_t value, cs_stream stream);

            void start_acquisition(cs_stream_id stream);

            void stop_stream(cs_stream_id stream);

            void stop_acquisition(cs_stream_id stream);

            bool select_channel(cs_stream_id stream);

            uint32_t read_from_buffer(std::vector<byte>& buffer, uint32_t index);

            std::vector<byte> send_hwm_to_device(std::vector<byte>& buffer);
            void set_rgb_ae_roi(uint32_t top, uint32_t left, uint32_t bottom, uint32_t right);

            std::vector<uint32_t> get_frame_rates(); 
            std::vector<uint32_t> get_frame_rates_from_control();

            uint32_t cs_pixel_format_to_native_pixel_format(std::string cs_format);

            std::vector<std::unique_ptr <std::thread>> _threads;
            platform::cs_device_info _device_info;
            power_state _power_state;
            std::vector<std::atomic<bool>> _is_capturing;
            std::mutex _power_lock, _stream_lock, _hwm_lock;
            uint8_t _number_of_streams;
            smcs::ICameraAPI _smcs_api;
            smcs::IDevice _connected_device;
            std::vector<frame_callback> _callbacks;
            std::string _device_version;
            cs_firmware_version _cs_firmware_version;
        };
    }

    class cs_sensor : public sensor_base
    {
    public:
        explicit cs_sensor(std::string name, std::shared_ptr<platform::cs_device> cs_device,
                           std::unique_ptr<frame_timestamp_reader> timestamp_reader, device* dev, cs_stream stream)
                : sensor_base(name, dev, (recommended_proccesing_blocks_interface*)this),
                  _timestamp_reader(std::move(timestamp_reader)),
                  _device(std::move(cs_device)),
                  _cs_stream_id(cs_stream_to_id(stream)),
                  _cs_stream(stream),
                  _user_count(0)
        {
            register_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP,     make_additional_data_parser(&frame_additional_data::backend_timestamp));
        }

        rs2_extension stream_to_frame_types(rs2_stream stream);

        virtual stream_profiles init_stream_profiles() override;

        void open(const stream_profiles& requests) override;

        void close() override;

        void start(frame_callback_ptr callback) override;

        void stop() override;

        template<class T>
        auto invoke_powered(T action)
        -> decltype(action(*static_cast<platform::cs_device*>(nullptr)))
        {
            power on(std::dynamic_pointer_cast<cs_sensor>(shared_from_this()));
            return action(*_device);
        }

        std::vector<platform::stream_profile> get_configuration() const { return _internal_config; }

        void register_pu(rs2_option id);

        void try_register_pu(rs2_option id);

    private:
        void acquire_power();

        void release_power();

        void reset_streaming();

        struct power
        {
            explicit power(std::weak_ptr<cs_sensor> owner)
                    : _owner(owner)
            {
                auto strong = _owner.lock();
                if (strong)
                {
                    strong->acquire_power();
                }
            }

            ~power()
            {
                if (auto strong = _owner.lock())
                {
                    try
                    {
                        strong->release_power();
                    }
                    catch (...) {}
                }
            }
        private:
            std::weak_ptr<cs_sensor> _owner;
        };

        std::unique_ptr<frame_timestamp_reader> _timestamp_reader;
        std::atomic<int> _user_count;
        std::mutex _power_lock;
        std::mutex _configure_lock;
        cs_stream _cs_stream;
        cs_stream_id _cs_stream_id;
        std::unique_ptr<power> _power;
        std::shared_ptr<platform::cs_device> _device;
    };

    class cs_pu_option : public option
    {
    public:
        void set(float value) override;

        float query() const override;

        option_range get_range() const override;

        bool is_enabled() const override
        {
            return true;
        }

        cs_pu_option(cs_sensor& ep, rs2_option id, cs_stream stream)
                : _ep(ep), _id(id), _stream(stream)
        {
        }

        cs_pu_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
                : _ep(ep), _id(id), _stream(stream), _description_per_value(description_per_value)
        {
        }

        const char* get_description() const override;

        const char* get_value_description(float val) const override
        {
            if (_description_per_value.find(val) != _description_per_value.end())
                return _description_per_value.at(val).c_str();
            return nullptr;
        }
        void enable_recording(std::function<void(const option &)> record_action) override
        {
            _record = record_action;
        }

    private:
        cs_sensor& _ep;
        cs_stream _stream;
        rs2_option _id;
        const std::map<float, std::string> _description_per_value;
        std::function<void(const option &)> _record = [](const option &) {};
    };
}

#endif //LIBREALSENSE2_CS_SENSOR_H
