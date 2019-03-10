//
// Created by marko on 09.03.19..
//

#pragma once

#include "context.h"
#include "device.h"
#include "core/debug.h"
#include "environment.h"

const uint16_t VID_SMARTEK_CAMERA           = 0x2479;

namespace librealsense
{
    const uint16_t UCC2592C_PID = 0x1013;

    class cs_camera;

    class cs_timestamp_reader : public frame_timestamp_reader
    {
    private:
        static const int pins = 2;
        mutable std::vector<int64_t> counter;
        std::shared_ptr<platform::time_service> _ts;
        mutable std::recursive_mutex _mtx;

    public:
        cs_timestamp_reader(void);

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
                platform::usb_device_info hwm)
            : device_info(ctx),
              _hwm(std::move(hwm))
        {}

        static std::vector<std::shared_ptr<device_info>> pick_cs_devices(
            std::shared_ptr<context> ctx,
            std::vector<platform::usb_device_info>& usb);

        platform::backend_device_group get_device_data() const override
        {
            return platform::backend_device_group({ _hwm });
        }

    private:
        platform::usb_device_info _hwm;
    };

    class cs_camera : public device
    {
    public:
        std::vector<tagged_profile> get_profiles_tags() const override
        {
            std::vector<tagged_profile> markers;
            markers.push_back({ RS2_STREAM_COLOR, -1, 1920, 1080, RS2_FORMAT_RGB8, 30, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
            return markers;
        };

        cs_camera(std::shared_ptr<context> ctx,
                  const platform::usb_device_info &hwm_device,
                  const platform::backend_device_group& group,
                  bool register_device_notifications);
    };

    class cs_sensor : public sensor_base
    {
    public:
        explicit cs_sensor(cs_camera* owner, std::shared_ptr<platform::usb_device> usb_device,
            std::unique_ptr<frame_timestamp_reader> timestamp_reader, std::shared_ptr<context> ctx)
            : sensor_base("RGB Camera", owner),
              _timestamp_reader(std::move(timestamp_reader)),
              _device(usb_device),
              _owner(owner)
        {
            register_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP,     make_additional_data_parser(&frame_additional_data::backend_timestamp));
        }

        stream_profiles init_stream_profiles() override
        {
            auto lock = environment::get_instance().get_extrinsics_graph().lock();

            //TODO
            //trebalo bi dohvatiti profile s kamere, trenutno su hardkodirani

            // tu sad ide stvar koju moram skuziti
            /*
             std::unordered_set<std::shared_ptr<video_stream_profile>> results;
        std::set<uint32_t> unregistered_formats;
        std::set<uint32_t> supported_formats;
        std::set<uint32_t> registered_formats;

        power on(std::dynamic_pointer_cast<uvc_sensor>(shared_from_this()));
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
             */

            /*
             * auto lock = environment::get_instance().get_extrinsics_graph().lock();

                auto results = uvc_sensor::init_stream_profiles();

                for (auto p : results)
                {
                    // Register stream types
                    if (p->get_stream_type() == RS2_STREAM_COLOR)
                    {
                        assign_stream(_owner->_color_stream, p);
                    }

                    // Register intrinsics
                    auto video = dynamic_cast<video_stream_profile_interface*>(p.get());

                    auto profile = to_profile(p.get());
                    std::weak_ptr<sr300_color_sensor> wp =
                        std::dynamic_pointer_cast<sr300_color_sensor>(this->shared_from_this());
                    video->set_intrinsics([profile, wp]()
                    {
                        auto sp = wp.lock();
                        if (sp)
                            return sp->get_intrinsics(profile);
                        else
                            return rs2_intrinsics{};
                    });
                }

                return results;
             *
             *
             * */
            /*auto lock = environment::get_instance().get_extrinsics_graph().lock();

            auto results = uvc_sensor::init_stream_profiles();

            for (auto p : results)
            {
                // Register stream types
                assign_stream(_default_stream, p);
                environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_default_stream, *p);
            }

            return results;*/

            stream_profiles a;

            return a;
        }

        void open(const stream_profiles& requests) override;

        void close() override;

        void start(frame_callback_ptr callback) override;

        void stop() override;

    private:
        const cs_camera* _owner;
        std::unique_ptr<frame_timestamp_reader> _timestamp_reader;
        std::shared_ptr<platform::usb_device> _device;

    };
}