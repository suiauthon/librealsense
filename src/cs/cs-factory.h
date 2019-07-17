//
// Created by marko on 21.05.19..
//

#ifndef LIBREALSENSE2_CS_FACTORY_H
#define LIBREALSENSE2_CS_FACTORY_H

#include "smcs_cpp/CameraSDK.h"
#include "smcs_cpp/IImageBitmap.h"
#include "stream.h"
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
                //printf("Ubijam cs device\n");
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

            std::vector<byte> send_hwm(std::vector<byte>& buffer);

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
            std::mutex _power_lock, _stream_lock, _hwm_lock;;
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

        virtual stream_profiles init_stream_profiles() override
        {
            auto lock = environment::get_instance().get_extrinsics_graph().lock();

            std::unordered_set<std::shared_ptr<video_stream_profile>> results;
            std::set<uint32_t> unregistered_formats;
            std::set<uint32_t> supported_formats;
            std::set<uint32_t> registered_formats;

            power on(std::dynamic_pointer_cast<cs_sensor>(shared_from_this()));

            if (_uvc_profiles.empty()){}
            _uvc_profiles = _device->get_profiles();

            for (auto&& p : _uvc_profiles)
            {
                supported_formats.insert(p.format);
                native_pixel_format pf{};
                if (try_get_pf(p, pf))
                {
                    for (auto&& unpacker : pf.unpackers)
                    {
                        for (auto&& output : unpacker.outputs)
                        {
                            auto profile = std::make_shared<video_stream_profile>(p);
                            auto res = output.stream_resolution({ p.width, p.height });
                            profile->set_dims(res.width, res.height);
                            profile->set_stream_type(output.stream_desc.type);
                            profile->set_stream_index(output.stream_desc.index);
                            profile->set_format(output.format);
                            profile->set_framerate(p.fps);
                            results.insert(profile);
                        }
                    }
                }
                else
                {
                    unregistered_formats.insert(p.format);
                }
            }

            if (unregistered_formats.size())
            {
                std::stringstream ss;
                ss << "Unregistered Media formats : [ ";
                for (auto& elem : unregistered_formats)
                {
                    uint32_t device_fourcc = reinterpret_cast<const big_endian<uint32_t>&>(elem);
                    char fourcc[sizeof(device_fourcc) + 1];
                    librealsense::copy(fourcc, &device_fourcc, sizeof(device_fourcc));
                    fourcc[sizeof(device_fourcc)] = 0;
                    ss << fourcc << " ";
                }

                ss << "]; Supported: [ ";
                for (auto& elem : registered_formats)
                {
                    uint32_t device_fourcc = reinterpret_cast<const big_endian<uint32_t>&>(elem);
                    char fourcc[sizeof(device_fourcc) + 1];
                    librealsense::copy(fourcc, &device_fourcc, sizeof(device_fourcc));
                    fourcc[sizeof(device_fourcc)] = 0;
                    ss << fourcc << " ";
                }
                ss << "]";
                LOG_WARNING(ss.str());
            }

            // Sort the results to make sure that the user will receive predictable deterministic output from the API
            stream_profiles res{ begin(results), end(results) };
            std::sort(res.begin(), res.end(), [](const std::shared_ptr<stream_profile_interface>& ap,
                                                 const std::shared_ptr<stream_profile_interface>& bp)
            {
                auto a = to_profile(ap.get());
                auto b = to_profile(bp.get());

                // stream == RS2_STREAM_COLOR && format == RS2_FORMAT_BGR8 element works around the fact that Y16 gets priority over BGR8 when both
                // are available for pipeline stream resolution
                auto at = std::make_tuple(a.stream, a.width, a.height, a.fps, a.stream == RS2_STREAM_COLOR && a.format == RS2_FORMAT_BGR8, a.format);
                auto bt = std::make_tuple(b.stream, b.width, b.height, b.fps, b.stream == RS2_STREAM_COLOR && b.format == RS2_FORMAT_BGR8, b.format);

                return at > bt;
            });

            return res;
        }

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
}

#endif //LIBREALSENSE2_CS_FACTORY_H
