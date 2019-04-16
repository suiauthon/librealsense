//
// Created by marko on 09.03.19..
//

#pragma once

#include "context.h"
#include "device.h"
#include "core/debug.h"
#include "environment.h"
#include "smcs_cpp/CameraSDK.h"
#include "smcs_cpp/IImageBitmap.h"
#include "stream.h"

namespace librealsense
{
    class cs_camera;
    class cs_sensor;

    namespace platform
    {
        class cs_device
        {
        public:
            explicit cs_device(platform::cs_device_info hwm)
                    : _device_info(std::move(hwm)),
                      _power_state(D3),
                      _is_connected(false),
                      _is_capturing(false),
                      _is_started(false),
                      _thread(nullptr),
                      _connected_device(NULL)
            {}

            power_state set_power_state(power_state state);

            void stream_on(std::function<void(const notification& n)> error_handler);

            void probe_and_commit(stream_profile profile, frame_callback callback);

            void close(stream_profile profile);

            bool connect(void);

            void disconnect(void);

            void start_callbacks();

            void stop_callbacks();

            void poll();

            power_state get_power_state() const { return _power_state; }

            bool get_pu(rs2_option opt, int32_t& value);

            bool set_pu(rs2_option opt, int32_t value);

            control_range get_pu_range(rs2_option option);

            std::vector<stream_profile> get_profiles();

            bool reset(void);

        protected:
            void prepare_capture_buffers();
            void stop_data_capture();
            void start_data_capture();
            void capture_loop();
            void set_format(stream_profile profile);

            std::function<void(const notification& n)> _error_handler;
            stream_profile _profile;

        private:
            uint32_t get_rs2_format(std::string format);
            std::string get_cs_param_name(rs2_option option);
            bool get_cs_param_min(rs2_option option, int32_t &value);
            bool get_cs_param_max(rs2_option option, int32_t &value);
            int32_t get_cs_param_step(rs2_option option);
            bool get_cs_param_value(rs2_option option, int32_t &value);
            bool set_cs_param(rs2_option option, int32_t value);

            //std::vector<std::shared_ptr<buffer>> _buffers;
            std::unique_ptr<std::thread> _thread;
            platform::cs_device_info _device_info;
            power_state _power_state;
            std::atomic<bool> _is_connected;
            std::atomic<bool> _is_capturing;
            std::atomic<bool> _is_started;
            std::mutex _power_lock;
            smcs::ICameraAPI _smcs_api;
            smcs::IDevice _connected_device;
            frame_callback _callback;
        };
    }

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

        cs_pu_option(cs_sensor& ep, rs2_option id)
                : _ep(ep), _id(id)
        {
        }

        cs_pu_option(cs_sensor& ep, rs2_option id, const std::map<float, std::string>& description_per_value)
                : _ep(ep), _id(id), _description_per_value(description_per_value)
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
        rs2_option _id;
        const std::map<float, std::string> _description_per_value;
        std::function<void(const option &)> _record = [](const option &) {};
    };

    class cs_timestamp_reader : public frame_timestamp_reader
    {
    private:
        mutable int64_t counter;
        std::shared_ptr<platform::time_service> _ts;
        mutable std::recursive_mutex _mtx;

    public:
        cs_timestamp_reader(std::shared_ptr<platform::time_service> ts);

        void reset() override;

        rs2_time_t get_frame_timestamp(const request_mapping& mode, const platform::frame_object& fo) override;

        unsigned long long get_frame_counter(const request_mapping & mode, const platform::frame_object& fo) const override;

        rs2_timestamp_domain get_frame_timestamp_domain(const request_mapping & mode, const platform::frame_object& fo) const override;
    };

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

    private:
        platform::cs_device_info _hwm;
    };

    class cs_sensor : public sensor_base
    {
    public:
        explicit cs_sensor(std::string name, std::shared_ptr<platform::cs_device> cs_device,
                           std::unique_ptr<frame_timestamp_reader> timestamp_reader, device* dev)
                : sensor_base(name, dev, (recommended_proccesing_blocks_interface*)this),
                  _timestamp_reader(std::move(timestamp_reader)),
                  _device(std::move(cs_device)),
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

                // stream == RS2_STREAM_COLOR && format == RS2_FORMAT_RGB8 element works around the fact that Y16 gets priority over RGB8 when both
                // are available for pipeline stream resolution
                auto at = std::make_tuple(a.stream, a.width, a.height, a.fps, a.stream == RS2_STREAM_COLOR && a.format == RS2_FORMAT_RGB8, a.format);
                auto bt = std::make_tuple(b.stream, b.width, b.height, b.fps, b.stream == RS2_STREAM_COLOR && b.format == RS2_FORMAT_RGB8, b.format);

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
        std::unique_ptr<power> _power;
        std::shared_ptr<platform::cs_device> _device;
    };

    class cs_camera : public device
    {
    public:
        std::vector<tagged_profile> get_profiles_tags() const override
        {
            std::vector<tagged_profile> markers;
            markers.push_back({ RS2_STREAM_ANY, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
            return markers;
        };

        cs_camera(std::shared_ptr<context> ctx,
                  const platform::cs_device_info &hwm_device,
                  const platform::backend_device_group& group,
                  bool register_device_notifications);

        cs_sensor& get_cs_sensor(size_t subdevice) { return dynamic_cast<cs_sensor&>(get_sensor(subdevice)); }

        void hardware_reset() override
        {
            get_cs_sensor(_color_device_idx).stop();
            //printf("Reset %d\n",_cs_device->reset());
            //get_cs_sensor(_color_device_idx).close();
        }

        class cs_color_sensor : public cs_sensor
        {
        public:
            explicit cs_color_sensor(cs_camera* owner, std::shared_ptr<platform::cs_device> cs_device,
                                     std::unique_ptr<frame_timestamp_reader> timestamp_reader,
                                     std::shared_ptr<context> ctx)
                    : cs_sensor("RGB Camera", cs_device, move(timestamp_reader), owner),
                      _owner(owner)
            {}

            stream_profiles init_stream_profiles() override
            {
                auto lock = environment::get_instance().get_extrinsics_graph().lock();

                auto results = cs_sensor::init_stream_profiles();

                for (auto p : results)
                {
                    // Register stream types
                    if (p->get_stream_type() == RS2_STREAM_COLOR)
                    {
                        assign_stream(_owner->_color_stream, p);
                    }
                    //assign_stream(_default_stream, p);
                    //TODO
                    //provjeriti
                    //environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_owner->_color_stream, *p);
                }

                return results;
            }
        private:
            const cs_camera* _owner;
        };

        class cs_depth_sensor : public cs_sensor
        {
        public:
            explicit cs_depth_sensor(cs_camera* owner, std::shared_ptr<platform::cs_device> cs_device,
                                     std::unique_ptr<frame_timestamp_reader> timestamp_reader,
                                     std::shared_ptr<context> ctx)
                    : cs_sensor("RGB Camera1", cs_device, move(timestamp_reader), owner),
                      _owner(owner)
            {}

            stream_profiles init_stream_profiles() override
            {
                auto lock = environment::get_instance().get_extrinsics_graph().lock();

                auto results = cs_sensor::init_stream_profiles();

                for (auto p : results)
                {
                    // Register stream types
                    if (p->get_stream_type() == RS2_STREAM_COLOR)
                    {
                        assign_stream(_owner->_depth_stream, p);
                    }
                    //assign_stream(_default_stream, p);
                    //TODO
                    //provjeriti
                    //environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_owner->_color_stream, *p);
                }

                return results;
            }
        private:
            const cs_camera* _owner;
        };

        std::shared_ptr<cs_sensor> create_color_device(std::shared_ptr<context> ctx,
                                                       std::shared_ptr<platform::cs_device> cs_device)
        {
            auto color_ep = std::make_shared<cs_color_sensor>(this, cs_device,
                                                              std::unique_ptr<cs_timestamp_reader>(new cs_timestamp_reader(environment::get_instance().get_time_service())),
                                                              ctx);

            color_ep->register_pixel_format(pf_raw8);

            /*color_ep->try_register_pu(RS2_OPTION_BRIGHTNESS);
            color_ep->try_register_pu(RS2_OPTION_CONTRAST);
            color_ep->try_register_pu(RS2_OPTION_HUE);
            color_ep->try_register_pu(RS2_OPTION_SATURATION);
            color_ep->try_register_pu(RS2_OPTION_SHARPNESS);*/
            color_ep->try_register_pu(RS2_OPTION_GAMMA);

            auto exposure_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_EXPOSURE);
            auto auto_exposure_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_ENABLE_AUTO_EXPOSURE);
            color_ep->register_option(RS2_OPTION_EXPOSURE, exposure_option);
            color_ep->register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_option);
            color_ep->register_option(RS2_OPTION_EXPOSURE,
                                      std::make_shared<auto_disabling_control>(
                                              exposure_option,
                                              auto_exposure_option));

            /*auto gain_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_GAIN);
            color_ep->register_option(RS2_OPTION_GAIN,
                                      std::make_shared<auto_disabling_control>(
                                              gain_option,
                                              auto_exposure_option));

            auto white_balance_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_WHITE_BALANCE);
            auto auto_white_balance_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
            color_ep->register_option(RS2_OPTION_WHITE_BALANCE, white_balance_option);
            color_ep->register_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, auto_white_balance_option);
            color_ep->register_option(RS2_OPTION_WHITE_BALANCE,
                                      std::make_shared<auto_disabling_control>(
                                              white_balance_option,
                                              auto_white_balance_option));*/

            return color_ep;
        }

        std::shared_ptr<cs_sensor> create_depth_device(std::shared_ptr<context> ctx,
                                                       std::shared_ptr<platform::cs_device> cs_device)
        {
            auto depth_ep = std::make_shared<cs_depth_sensor>(this, cs_device,
                                                              std::unique_ptr<cs_timestamp_reader>(new cs_timestamp_reader(environment::get_instance().get_time_service())),
                                                              ctx);

            depth_ep->register_pixel_format(pf_raw8);

            /*color_ep->try_register_pu(RS2_OPTION_BRIGHTNESS);
            color_ep->try_register_pu(RS2_OPTION_CONTRAST);
            color_ep->try_register_pu(RS2_OPTION_HUE);
            color_ep->try_register_pu(RS2_OPTION_SATURATION);
            color_ep->try_register_pu(RS2_OPTION_SHARPNESS);*/
            depth_ep->try_register_pu(RS2_OPTION_GAMMA);

            auto exposure_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_EXPOSURE);
            auto auto_exposure_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_ENABLE_AUTO_EXPOSURE);
            depth_ep->register_option(RS2_OPTION_EXPOSURE, exposure_option);
            depth_ep->register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_option);
            depth_ep->register_option(RS2_OPTION_EXPOSURE,
                                      std::make_shared<auto_disabling_control>(
                                              exposure_option,
                                              auto_exposure_option));

            /*auto gain_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_GAIN);
            color_ep->register_option(RS2_OPTION_GAIN,
                                      std::make_shared<auto_disabling_control>(
                                              gain_option,
                                              auto_exposure_option));

            auto white_balance_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_WHITE_BALANCE);
            auto auto_white_balance_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
            color_ep->register_option(RS2_OPTION_WHITE_BALANCE, white_balance_option);
            color_ep->register_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, auto_white_balance_option);
            color_ep->register_option(RS2_OPTION_WHITE_BALANCE,
                                      std::make_shared<auto_disabling_control>(
                                              white_balance_option,
                                              auto_white_balance_option));*/

            return depth_ep;
        }
    private:
        uint8_t _color_device_idx;
        uint8_t _depth_device_idx;
        std::shared_ptr<stream_interface> _color_stream;
        std::shared_ptr<stream_interface> _depth_stream;
        std::shared_ptr<platform::cs_device> _cs_device;
    };
}