//
// Created by marko on 09.03.19..
//

#pragma once

#include "cs/cs-factory.h"
#include "device.h"
#include "core/debug.h"
#include "environment.h"
#include "stream.h"

namespace librealsense
{
    class cs_camera;
    class cs_sensor;

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

    class cs_sensor : public sensor_base
    {
    public:
        explicit cs_sensor(std::string name, std::shared_ptr<platform::cs_device> cs_device,
                           std::unique_ptr<frame_timestamp_reader> timestamp_reader, device* dev, cs_stream stream)
                : sensor_base(name, dev, (recommended_proccesing_blocks_interface*)this),
                  _timestamp_reader(std::move(timestamp_reader)),
                  _device(std::move(cs_device)),
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
        cs_stream _cs_stream;
        std::unique_ptr<power> _power;
        std::shared_ptr<platform::cs_device> _device;
    };

    class cs_color : public virtual device
    {
    public:
        cs_color(std::shared_ptr<context> ctx,
                 const platform::backend_device_group& group,
                 bool register_device_notifications)
                : device(ctx, group, register_device_notifications),
                  _color_stream(new stream(RS2_STREAM_COLOR))
        {};

        std::shared_ptr<cs_sensor> create_color_device(std::shared_ptr<context> ctx,
                                                       std::shared_ptr<platform::cs_device> cs_device);
    protected:
        std::shared_ptr<stream_interface> _color_stream;
        uint8_t _color_device_idx;

    private:
        friend class cs_color_sensor;

        //lazy<std::vector<uint8_t>> _color_calib_table_raw;
        //std::shared_ptr<lazy<rs2_extrinsics>> _color_extrinsic;
    };

    class cs_depth : public virtual device
    {
    public:
        cs_depth(std::shared_ptr<context> ctx,
                 const platform::backend_device_group& group,
                 bool register_device_notifications)
                : device(ctx, group, register_device_notifications),
                  _depth_stream(new stream(RS2_STREAM_DEPTH))
        {};

        std::shared_ptr<cs_sensor> create_depth_device(std::shared_ptr<context> ctx,
                                                       std::shared_ptr<platform::cs_device> cs_device);
    protected:
        std::shared_ptr<stream_interface> _depth_stream;
        uint8_t _depth_device_idx;

    private:
        friend class cs_depth_sensor;

        //lazy<std::vector<uint8_t>> _depth_calib_table_raw;
        //std::shared_ptr<lazy<rs2_extrinsics>> _depth_extrinsic;
    };

    class cs_mono : public virtual device
    {
    public:
        cs_mono(std::shared_ptr<context> ctx,
                 const platform::backend_device_group& group,
                 bool register_device_notifications)
                : device(ctx, group, register_device_notifications),
                  _fisheye_stream(new stream(RS2_STREAM_FISHEYE))
        {};

        std::shared_ptr<cs_sensor> create_mono_device(std::shared_ptr<context> ctx,
                                                       std::shared_ptr<platform::cs_device> cs_device);
    protected:
        std::shared_ptr<stream_interface> _fisheye_stream;
        uint8_t _mono_device_idx;

    private:
        friend class cs_mono_sensor;

        //lazy<std::vector<uint8_t>> _depth_calib_table_raw;
        //std::shared_ptr<lazy<rs2_extrinsics>> _depth_extrinsic;
    };

    class cs_camera: public virtual device,
                     public debug_interface
    {
    public:
        cs_camera(std::shared_ptr<context> ctx,
                  const platform::cs_device_info &hwm_device,
                  const platform::backend_device_group& group,
                  bool register_device_notifications);

        std::vector<uint8_t> send_receive_raw_data(const std::vector<uint8_t>& input) override;
        void create_snapshot(std::shared_ptr<debug_interface>& snapshot) const override;
        void enable_recording(std::function<void(const debug_interface&)> record_action) override;

        //TODO implementirati ovo za svaku kameru posebno, ne u cs_camera nego u onoj klasi koja nasljeduje cs_cameru
        //void hardware_reset() override
        //{
        /*if (get_cs_sensor(_color_device_idx).is_streaming())
            get_cs_sensor(_color_device_idx).stop();*/

        //printf("Reset %d\n",_cs_device->reset());


        /*if (get_cs_sensor(_color_device_idx)._is_opened
                    get_cs_sensor(_color_device_idx).close();*/
        //}

    private:
    };

    class CSMono_camera: public cs_mono,
                         public cs_camera
    {
    public:
        CSMono_camera(std::shared_ptr<context> ctx,
                      const platform::cs_device_info &hwm_device,
                      const platform::backend_device_group& group,
                      bool register_device_notifications);

        std::shared_ptr<matcher> create_matcher(const frame_holder& frame) const override;

        std::vector<tagged_profile> get_profiles_tags() const override;

        cs_sensor& get_cs_sensor(size_t subdevice) { return dynamic_cast<cs_sensor&>(get_sensor(subdevice)); }

    private:
        std::shared_ptr<platform::cs_device> _cs_device;
    };

    class D435e_camera : public cs_color,
                         public cs_depth,
                         public cs_camera
    {
    public:
        D435e_camera(std::shared_ptr<context> ctx,
                     const platform::cs_device_info &hwm_device,
                     const platform::backend_device_group& group,
                     bool register_device_notifications);

        std::shared_ptr<matcher> create_matcher(const frame_holder& frame) const override;

        std::vector<tagged_profile> get_profiles_tags() const override;

        cs_sensor& get_cs_sensor(size_t subdevice) { return dynamic_cast<cs_sensor&>(get_sensor(subdevice)); }

    private:
        std::shared_ptr<platform::cs_device> _cs_device;
    };

    class cs_color_sensor : public cs_sensor
    {
    public:
        explicit cs_color_sensor(cs_color* owner, std::shared_ptr<platform::cs_device> cs_device,
                                 std::unique_ptr<frame_timestamp_reader> timestamp_reader,
                                 std::shared_ptr<context> ctx)
                : cs_sensor("RGB Camera", cs_device, move(timestamp_reader), owner, CS_STREAM_COLOR),
                  _owner(owner)
        {};

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

                //TODO
                //provjeriti
                //environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_owner->_color_stream, *p);
            }

            return results;
        };

        //rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        //processing_blocks get_recommended_processing_blocks() const override;
    private:
        const cs_color* _owner;
    };

    class cs_depth_sensor : public cs_sensor
    {
    public:
        explicit cs_depth_sensor(cs_depth* owner, std::shared_ptr<platform::cs_device> cs_device,
                                 std::unique_ptr<frame_timestamp_reader> timestamp_reader,
                                 std::shared_ptr<context> ctx)
                : cs_sensor("Depth Sensor", cs_device, move(timestamp_reader), owner, CS_STREAM_DEPTH),
                  _owner(owner)
        {};

        stream_profiles init_stream_profiles() override
        {
            auto lock = environment::get_instance().get_extrinsics_graph().lock();

            auto results = cs_sensor::init_stream_profiles();

            for (auto p : results)
            {
                // Register stream types
                if (p->get_stream_type() == RS2_STREAM_DEPTH)
                {
                    assign_stream(_owner->_depth_stream, p);
                }

                //TODO
                //provjeriti
                //environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_owner->_color_stream, *p);
            }

            return results;
        }

        //rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        //processing_blocks get_recommended_processing_blocks() const override;
    private:
        const cs_depth* _owner;
    };

    class cs_mono_sensor : public cs_sensor
    {
    public:
        explicit cs_mono_sensor(cs_mono* owner, std::shared_ptr<platform::cs_device> cs_device,
                                 std::unique_ptr<frame_timestamp_reader> timestamp_reader,
                                 std::shared_ptr<context> ctx)
                : cs_sensor("MONO Camera", cs_device, move(timestamp_reader), owner, CS_STREAM_MONO),
                  _owner(owner)
        {};

        stream_profiles init_stream_profiles() override
        {
            auto lock = environment::get_instance().get_extrinsics_graph().lock();

            auto results = cs_sensor::init_stream_profiles();

            for (auto p : results)
            {
                // Register stream types
                if (p->get_stream_type() == RS2_STREAM_FISHEYE)
                {
                    assign_stream(_owner->_fisheye_stream, p);
                }

                //TODO
                //provjeriti
                //environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_owner->_color_stream, *p);
            }

            return results;
        };

        //rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        //processing_blocks get_recommended_processing_blocks() const override;
    private:
        const cs_mono* _owner;
    };
}