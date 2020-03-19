// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#pragma once

#include "cs/cs-sensor.h"
#include "device.h"
#include "ds5/ds5-private.h"
#include "global_timestamp_reader.h"
#include "core/advanced_mode.h"
#include <map>
#include "stream.h"
#include "fw-update/fw-update-device-interface.h"

namespace librealsense
{
    std::map<uint32_t, rs2_format> cs_depth_fourcc_to_rs2_format = {
            {rs_fourcc('Y','U','Y','2'), RS2_FORMAT_YUYV},
            {rs_fourcc('Y','U','Y','V'), RS2_FORMAT_YUYV},
            {rs_fourcc('U','Y','V','Y'), RS2_FORMAT_UYVY},
            {rs_fourcc('G','R','E','Y'), RS2_FORMAT_Y8},
            {rs_fourcc('Y','8','I',' '), RS2_FORMAT_Y8I},
            {rs_fourcc('W','1','0',' '), RS2_FORMAT_W10},
            {rs_fourcc('Y','1','6',' '), RS2_FORMAT_Y16},
            {rs_fourcc('Y','1','2','I'), RS2_FORMAT_Y12I},
            {rs_fourcc('Z','1','6',' '), RS2_FORMAT_Z16},
            {rs_fourcc('Z','1','6','H'), RS2_FORMAT_Z16H},
            {rs_fourcc('R','G','B','2'), RS2_FORMAT_BGR8}

    };
    std::map<uint32_t, rs2_stream> cs_depth_fourcc_to_rs2_stream = {
            {rs_fourcc('Y','U','Y','2'), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','U','Y','V'), RS2_STREAM_INFRARED},
            {rs_fourcc('U','Y','V','Y'), RS2_STREAM_INFRARED},
            {rs_fourcc('G','R','E','Y'), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','8','I',' '), RS2_STREAM_INFRARED},
            {rs_fourcc('W','1','0',' '), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','1','6',' '), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','1','2','I'), RS2_STREAM_INFRARED},
            {rs_fourcc('R','G','B','2'), RS2_STREAM_INFRARED},
            {rs_fourcc('Z','1','6',' '), RS2_STREAM_DEPTH},
            {rs_fourcc('Z','1','6','H'), RS2_STREAM_DEPTH}
    };

    std::map<uint32_t, rs2_format> cs_color_fourcc_to_rs2_format = {
            {rs_fourcc('Y','U','Y','2'), RS2_FORMAT_YUYV},
            {rs_fourcc('Y','U','Y','V'), RS2_FORMAT_YUYV},
            {rs_fourcc('U','Y','V','Y'), RS2_FORMAT_UYVY},
            {rs_fourcc('M','J','P','G'), RS2_FORMAT_MJPEG},
            {rs_fourcc('B','Y','R','2'), RS2_FORMAT_RAW16}
    };
    std::map<uint32_t, rs2_stream> cs_color_fourcc_to_rs2_stream = {
            {rs_fourcc('Y','U','Y','2'), RS2_STREAM_COLOR},
            {rs_fourcc('Y','U','Y','V'), RS2_STREAM_COLOR},
            {rs_fourcc('U','Y','V','Y'), RS2_STREAM_COLOR},
            {rs_fourcc('B','Y','R','2'), RS2_STREAM_COLOR},
            {rs_fourcc('M','J','P','G'), RS2_STREAM_COLOR},
    };

    class cs_auto_exposure_roi_method : public region_of_interest_method
    {
    public:
        explicit cs_auto_exposure_roi_method(const hw_monitor& hwm,
                                             ds::fw_cmd cmd = ds::fw_cmd::SETAEROI);

        void set(const region_of_interest& roi) override;
        region_of_interest get() const override;
    private:
        const ds::fw_cmd _cmd;
        const hw_monitor& _hw_monitor;
    };

    class cs_depth_scale_option : public option, public observable_option
    {
    public:
        cs_depth_scale_option(hw_monitor& hwm);
        virtual ~cs_depth_scale_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Number of meters represented by a single depth unit";
        }
        void enable_recording(std::function<void(const option &)> record_action)
        {
            _record_action = record_action;
        }

    private:
        ds::depth_table_control get_depth_table(ds::advanced_query_mode mode) const;
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
    };

    class cs_color : public virtual device,  public global_time_interface
    {
    public:
        cs_color(std::shared_ptr<context> ctx,
                 const platform::backend_device_group& group,
                 bool register_device_notifications)
                : device(ctx, group, register_device_notifications),
                  _color_stream(new stream(RS2_STREAM_COLOR))
        {};

        std::shared_ptr<synthetic_sensor> create_color_device(std::shared_ptr<context> ctx,
                                                              std::shared_ptr<platform::cs_device> cs_device);

        cs_sensor& get_color_sensor() { return dynamic_cast<cs_sensor&>(get_sensor(_color_device_idx)); }

        void color_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group);

        virtual double get_device_time_ms() override;
    protected:
        std::vector<uint8_t> get_raw_calibration_table(ds::calibration_table_id table_id) const;

        std::shared_ptr<stream_interface> _color_stream;
        uint8_t _color_device_idx;

        friend class cs_color_sensor;

        std::shared_ptr<hw_monitor> _hw_monitor;

        lazy<std::vector<uint8_t>> _color_calib_table_raw;
        std::shared_ptr<lazy<rs2_extrinsics>> _color_extrinsic;
    };

    class cs_depth : public virtual device, public debug_interface, public global_time_interface, public updatable //public auto_calibrated
    {
    public:
        cs_depth(std::shared_ptr<context> ctx,
                 const platform::backend_device_group& group,
                 bool register_device_notifications)
                : device(ctx, group, register_device_notifications),
                  global_time_interface(),
                  //auto_calibrated(_hw_monitor),
                  _depth_stream(new stream(RS2_STREAM_DEPTH)),
                  _left_ir_stream(new stream(RS2_STREAM_INFRARED, 1)),
                  _right_ir_stream(new stream(RS2_STREAM_INFRARED, 2)),
                  _device_capabilities(ds::d400_caps::CAP_UNDEFINED)
        {
        }

        std::shared_ptr<synthetic_sensor> create_depth_device(std::shared_ptr<context> ctx,
                                                              std::shared_ptr<platform::cs_device> cs_device);

        synthetic_sensor& get_depth_sensor() { return dynamic_cast<synthetic_sensor&>(get_sensor(_depth_device_idx)); }
        cs_sensor& get_raw_depth_sensor() { return dynamic_cast<cs_sensor&>(get_sensor(_depth_device_idx)); };
        std::vector<uint8_t> send_receive_raw_data(const std::vector<uint8_t>& input) override;

        void create_snapshot(std::shared_ptr<debug_interface>& snapshot) const override;
        void enable_recording(std::function<void(const debug_interface&)> record_action) override;

        void depth_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group);

        virtual double get_device_time_ms() override;
        void enter_update_state() const override;
        std::vector<uint8_t> backup_flash(update_progress_callback_ptr callback) override;
        void update_flash(const std::vector<uint8_t>& image, update_progress_callback_ptr callback, int update_mode) override;

    protected:
        float get_stereo_baseline_mm() const;
        std::vector<uint8_t> get_raw_calibration_table(ds::calibration_table_id table_id) const;
        std::vector<uint8_t> get_new_calibration_table() const;
        bool is_camera_in_advanced_mode() const;
        processing_blocks get_cs_depth_recommended_proccesing_blocks() const;

        ds::d400_caps  parse_device_capabilities(const uint16_t pid) const;

        std::shared_ptr<stream_interface> _depth_stream;
        std::shared_ptr<stream_interface> _left_ir_stream;
        std::shared_ptr<stream_interface> _right_ir_stream;

        uint8_t _depth_device_idx;

        friend class cs_depth_sensor;

        std::shared_ptr<hw_monitor> _hw_monitor;
		std::shared_ptr<platform::cs_device> _cs_device;
        firmware_version            _fw_version;
        firmware_version            _recommended_fw_version;
        ds::d400_caps _device_capabilities;

        lazy<std::vector<uint8_t>> _depth_calib_table_raw;
        lazy<std::vector<uint8_t>> _new_calib_table_raw;

        std::shared_ptr<lazy<rs2_extrinsics>> _depth_extrinsic;
    };

    class cs_color_sensor : public synthetic_sensor,
                            public video_sensor_interface,
                            public roi_sensor_base,
                            public color_sensor
    {
    public:
        explicit cs_color_sensor(cs_color* owner,
                                 std::shared_ptr<cs_sensor> cs_sensor)
                : synthetic_sensor("RGB Camera", cs_sensor, owner, cs_color_fourcc_to_rs2_format, cs_color_fourcc_to_rs2_stream),
                  _owner(owner)
        {};

        stream_profiles init_stream_profiles() override;
        rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        processing_blocks get_recommended_processing_blocks() const override;
    private:
        const cs_color* _owner;
    };

    class cs_depth_sensor : public synthetic_sensor,
                            public roi_sensor_base,
                            public video_sensor_interface,
                            public depth_stereo_sensor
    {
    public:
        explicit cs_depth_sensor(cs_depth* owner,
                                 std::shared_ptr<cs_sensor> cs_sensor)
                : synthetic_sensor("Stereo Module", cs_sensor, owner, cs_depth_fourcc_to_rs2_format, cs_depth_fourcc_to_rs2_stream),
                  _owner(owner),
                  _depth_units(-1)
        {};
        //TODO zamijeniti stereo module s macro
        stream_profiles init_stream_profiles() override;
        void open(const stream_profiles& requests) override;
        rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        processing_blocks get_recommended_processing_blocks() const override;

        float get_depth_scale() const override
        {
            if (_depth_units < 0)
                _depth_units = get_option(RS2_OPTION_DEPTH_UNITS).query();
            return _depth_units;
        };

        void set_depth_scale(float val){ _depth_units = val; };
        float get_stereo_baseline_mm() const override { return _owner->get_stereo_baseline_mm(); }
        void enable_recording(std::function<void(const depth_stereo_sensor&)> recording_function) override;
        void enable_recording(std::function<void(const depth_sensor&)> recording_function) override;
        void create_snapshot(std::shared_ptr<depth_stereo_sensor>& snapshot) const override;
        void create_snapshot(std::shared_ptr<depth_sensor>& snapshot) const override;
    private:
        const cs_depth* _owner;
        mutable std::atomic<float> _depth_units;
    };

	class cs_external_sync_mode : public option
	{
	public:
		cs_external_sync_mode(hw_monitor& hwm, cs_depth_sensor& depth);
		virtual ~cs_external_sync_mode() = default;
		virtual void set(float value) override;
		virtual float query() const override;
		virtual option_range get_range() const override;
		virtual bool is_enabled() const override { return true; }

		const char* get_description() const override
		{
			return "Inter-camera synchronization mode: 0:Default, 1:Master, 2:Slave, 3:External Trigger";
		}
		void enable_recording(std::function<void(const option &)> record_action) override
		{
			_record_action = record_action;
		}
	private:
		std::function<void(const option &)> _record_action = [](const option&) {};
		lazy<option_range> _range;
		hw_monitor& _hwm;
		cs_depth_sensor& _depth;
	};

	class cs_external_sync_mode_color : public option
	{
	public:
		cs_external_sync_mode_color(cs_color_sensor& color);
		virtual ~cs_external_sync_mode_color() = default;
		virtual void set(float value) override;
		virtual float query() const override;
		virtual option_range get_range() const override;
		virtual bool is_enabled() const override { return true; }

		const char* get_description() const override
		{
			return "Inter-camera synchronization mode: 0:Default, 1:External Trigger";
		}
		void enable_recording(std::function<void(const option &)> record_action) override
		{
			_record_action = record_action;
		}
	private:
		std::function<void(const option &)> _record_action = [](const option&) {};
		lazy<option_range> _range;
		cs_color_sensor& _color;
	};

}

