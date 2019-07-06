//
// Created by marko on 09.03.19..
//

#pragma once

#include "cs/cs-factory.h"
#include "cs/cs-hw-monitor.h"
#include "device.h"
#include "core/debug.h"
#include "ds5/ds5-private.h"

namespace librealsense
{
    class cs_camera;

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

    class cs_auto_exposure_roi_method : public region_of_interest_method
    {
    public:
        explicit cs_auto_exposure_roi_method(cs_sensor& ep, cs_stream stream);

        void set(const region_of_interest& roi) override;
        region_of_interest get() const override;
    private:
        cs_sensor& _ep;
        cs_stream _stream;
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

    class cs_depth : public virtual device, public debug_interface
    {
    public:
        cs_depth(std::shared_ptr<context> ctx,
                 const platform::backend_device_group& group,
                 bool register_device_notifications)
                : device(ctx, group, register_device_notifications),
                  _depth_stream(new stream(RS2_STREAM_DEPTH)),
                  _device_capabilities(ds::d400_caps::CAP_UNDEFINED)
        {}

        std::shared_ptr<cs_sensor> create_depth_device(std::shared_ptr<context> ctx,
                                                       std::shared_ptr<platform::cs_device> cs_device);

        cs_sensor& get_depth_sensor() { return dynamic_cast<cs_sensor&>(get_sensor(_depth_device_idx)); }

        std::vector<uint8_t> send_receive_raw_data(const std::vector<uint8_t>& input) override;

        void create_snapshot(std::shared_ptr<debug_interface>& snapshot) const override;
        void enable_recording(std::function<void(const debug_interface&)> record_action) override;

        void depth_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group);

    protected:
        float get_stereo_baseline_mm() const;
        std::vector<uint8_t> get_raw_calibration_table(ds::calibration_table_id table_id) const;
        bool is_camera_in_advanced_mode() const;
        processing_blocks get_cs_depth_recommended_proccesing_blocks() const;

        ds::d400_caps  parse_device_capabilities() const;

        std::shared_ptr<stream_interface> _depth_stream;
        uint8_t _depth_device_idx;

    private:
        friend class cs_depth_sensor;

        std::shared_ptr<cs_hw_monitor> _hw_monitor;
        firmware_version            _fw_version;
        firmware_version            _recommended_fw_version;
        ds::d400_caps _device_capabilities;

        lazy<std::vector<uint8_t>> _depth_calib_table_raw;
        std::shared_ptr<lazy<rs2_extrinsics>> _depth_extrinsic;
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

    private:
    };

    class CSMono_camera: public cs_mono
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
                         public cs_depth
    {
    public:
        D435e_camera(std::shared_ptr<context> ctx,
                     const platform::cs_device_info &hwm_device,
                     const platform::backend_device_group& group,
                     bool register_device_notifications);

        std::shared_ptr<matcher> create_matcher(const frame_holder& frame) const override;

        std::vector<tagged_profile> get_profiles_tags() const override;

        cs_sensor& get_cs_sensor(size_t subdevice) { return dynamic_cast<cs_sensor&>(get_sensor(subdevice)); }

        void hardware_reset() override
        {
            if (get_cs_sensor(_color_device_idx).is_streaming()) {
                get_cs_sensor(_color_device_idx).stop();
                get_cs_sensor(_color_device_idx).close();
            }

            if (get_cs_sensor(_depth_device_idx).is_streaming()) {
                get_cs_sensor(_depth_device_idx).stop();
                get_cs_sensor(_depth_device_idx).close();
            }

            _cs_device->reset();
        }

    private:
        std::shared_ptr<platform::cs_device> _cs_device;
    };

    class cs_color_sensor : public cs_sensor,
                            public roi_sensor_base
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
        processing_blocks get_recommended_processing_blocks() const override;
    private:
        const cs_color* _owner;
    };

    class cs_depth_sensor : public cs_sensor,
                            public roi_sensor_base,
                            public video_sensor_interface,
                            public depth_stereo_sensor
    {
    public:
        explicit cs_depth_sensor(cs_depth* owner, std::shared_ptr<platform::cs_device> cs_device,
                                 std::unique_ptr<frame_timestamp_reader> timestamp_reader,
                                 std::shared_ptr<context> ctx)
                : cs_sensor("Stereo Module", cs_device, move(timestamp_reader), owner, CS_STREAM_DEPTH),
                  _owner(owner),
                  _depth_units(-1)
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
                auto vid_profile = dynamic_cast<video_stream_profile_interface*>(p.get());

                // Register intrinsics
                if (p->get_format() != RS2_FORMAT_Y16) // Y16 format indicate unrectified images, no intrinsics are available for these
                {
                    auto profile = to_profile(p.get());
                    std::weak_ptr<cs_depth_sensor> wp =
                            std::dynamic_pointer_cast<cs_depth_sensor>(this->shared_from_this());
                    vid_profile->set_intrinsics([profile, wp]()
                                                {
                                                    auto sp = wp.lock();
                                                    if (sp)
                                                        return sp->get_intrinsics(profile);
                                                    else
                                                        return rs2_intrinsics{};
                                                });
                }

                //TODO
                //provjeriti
                environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_owner->_depth_stream, *p);
            }

            return results;
        }

        rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        processing_blocks get_recommended_processing_blocks() const override;
        float get_depth_scale() const override { return 0.001; }
        void set_depth_scale(float val){ _depth_units = val; }
        float get_stereo_baseline_mm() const override { return _owner->get_stereo_baseline_mm(); }
        void enable_recording(std::function<void(const depth_stereo_sensor&)> recording_function) override;
        void enable_recording(std::function<void(const depth_sensor&)> recording_function) override;
        void create_snapshot(std::shared_ptr<depth_stereo_sensor>& snapshot) const override;
        void create_snapshot(std::shared_ptr<depth_sensor>& snapshot) const override;
    private:
        const cs_depth* _owner;
        mutable std::atomic<float> _depth_units;
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