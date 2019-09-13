//
// Created by marko on 09.03.19..
//

#include "cs-device.h"
#include "cs-timestamp.h"

#include "proc/decimation-filter.h"
#include "proc/threshold.h"
#include "proc/disparity-transform.h"
#include "proc/spatial-filter.h"
#include "proc/temporal-filter.h"
#include "proc/hole-filling-filter.h"
#include "../common/fw/firmware-version.h"

namespace librealsense
{
    cs_auto_exposure_roi_method::cs_auto_exposure_roi_method(const hw_monitor& hwm,
                                                             ds::fw_cmd cmd)
            : _hw_monitor(hwm), _cmd(cmd) {}

    void cs_auto_exposure_roi_method::set(const region_of_interest& roi)
    {
        command cmd(_cmd);
        cmd.param1 = roi.min_y;
        cmd.param2 = roi.max_y;
        cmd.param3 = roi.min_x;
        cmd.param4 = roi.max_x;
        _hw_monitor.send(cmd);
    }

    region_of_interest cs_auto_exposure_roi_method::get() const
    {
        region_of_interest roi;
        command cmd(_cmd + 1);
        auto res = _hw_monitor.send(cmd);

        if (res.size() < 4 * sizeof(uint16_t))
        {
            throw std::runtime_error("Invalid result size!");
        }

        auto words = reinterpret_cast<uint16_t*>(res.data());

        roi.min_y = words[0];
        roi.max_y = words[1];
        roi.min_x = words[2];
        roi.max_x = words[3];

        return roi;
    }

    ds::depth_table_control cs_depth_scale_option::get_depth_table(ds::advanced_query_mode mode) const
    {
        command cmd(ds::GET_ADV);
        cmd.param1 = ds::etDepthTableControl;
        cmd.param2 = mode;
        auto res = _hwm.send(cmd);

        if (res.size() < sizeof(ds::depth_table_control))
            throw std::runtime_error("Not enough bytes returned from the firmware!");

        auto table = (const ds::depth_table_control*)res.data();
        return *table;
    }

    cs_depth_scale_option::cs_depth_scale_option(hw_monitor& hwm)
            : _hwm(hwm)
    {
        _range = [this]()
        {
            auto min = get_depth_table(ds::GET_MIN);
            auto max = get_depth_table(ds::GET_MAX);
            return option_range{ (float)(0.000001 * min.depth_units),
                                 (float)(0.000001 * max.depth_units),
                                 0.000001f, 0.001f };
        };
    }

    void cs_depth_scale_option::set(float value)
    {
        command cmd(ds::SET_ADV);
        cmd.param1 = ds::etDepthTableControl;

        auto depth_table = get_depth_table(ds::GET_VAL);
        depth_table.depth_units = static_cast<uint32_t>(1000000 * value);
        auto ptr = (uint8_t*)(&depth_table);
        cmd.data = std::vector<uint8_t>(ptr, ptr + sizeof(ds::depth_table_control));

        _hwm.send(cmd);
        _record_action(*this);
        notify(value);
    }

    float cs_depth_scale_option::query() const
    {
        auto table = get_depth_table(ds::GET_VAL);
        return (float)(0.000001 * (float)table.depth_units);
    }

    option_range cs_depth_scale_option::get_range() const
    {
        return *_range;
    }

    void cs_color::color_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group)
    {
        using namespace ds;

        _hw_monitor = std::make_shared<hw_monitor>(&get_color_sensor());

        _color_calib_table_raw = [this]() { return get_raw_calibration_table(rgb_calibration_id); };
        _color_extrinsic = std::make_shared<lazy<rs2_extrinsics>>([this]() { return from_pose(get_color_stream_extrinsic(*_color_calib_table_raw)); });

        register_stream_to_extrinsic_group(*_color_stream, 0);

        //auto& color_ep = get_color_sensor();

        /*roi_sensor_interface* roi_sensor;
        if (roi_sensor = dynamic_cast<roi_sensor_interface*>(&color_ep))
            roi_sensor->set_roi_method(std::make_shared<cs_auto_exposure_roi_method>(*_hw_monitor, ds::fw_cmd::SETRGBAEROI));*/
    }

    void cs_depth::depth_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group)
    {
        using namespace ds;

        _hw_monitor = std::make_shared<hw_monitor>(&get_depth_sensor());

        _depth_extrinsic = std::make_shared<lazy<rs2_extrinsics>>([this]()
                {
                    rs2_extrinsics ext = identity_matrix();
                    auto table = check_calib<coefficients_table>(*_depth_calib_table_raw);
                    ext.translation[0] = 0.001f * table->baseline; // mm to meters
                    return ext;
                });

        environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_depth_stream, *_left_ir_stream);
        environment::get_instance().get_extrinsics_graph().register_extrinsics(*_depth_stream, *_right_ir_stream, _depth_extrinsic);

        register_stream_to_extrinsic_group(*_depth_stream, 0);
        register_stream_to_extrinsic_group(*_left_ir_stream, 0);
        register_stream_to_extrinsic_group(*_right_ir_stream, 0);

        _depth_calib_table_raw = [this]() { return get_raw_calibration_table(coefficients_table_id); };
        _new_calib_table_raw = [this]() { return get_new_calibration_table(); };

        auto pid = group.cs_devices.front().vid;

        std::vector<uint8_t> gvd_buff(HW_MONITOR_BUFFER_SIZE);
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);
        // fooling tests recordings - don't remove
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);

        auto optic_serial = _hw_monitor->get_module_serial_string(gvd_buff, module_serial_offset);
        auto asic_serial = _hw_monitor->get_module_serial_string(gvd_buff, module_asic_serial_offset);
        auto fwv = _hw_monitor->get_firmware_version_string(gvd_buff, camera_fw_version_offset);
        _fw_version = firmware_version(fwv);

        _recommended_fw_version = firmware_version(D4XX_RECOMMENDED_FIRMWARE_VERSION);
        if (_fw_version >= firmware_version("5.10.4.0"))
            _device_capabilities = parse_device_capabilities(pid);

        auto& depth_ep = get_depth_sensor();
        auto advanced_mode = is_camera_in_advanced_mode();

        using namespace platform;

        if (advanced_mode)
        {
            depth_ep.register_pixel_format(pf_y8i); // L+R
            depth_ep.register_pixel_format(pf_y12i); // L+R - Calibration not rectified
        }

        roi_sensor_interface* roi_sensor;
        if (roi_sensor = dynamic_cast<roi_sensor_interface*>(&depth_ep))
            roi_sensor->set_roi_method(std::make_shared<cs_auto_exposure_roi_method>(*_hw_monitor));

        depth_ep.register_option(RS2_OPTION_STEREO_BASELINE, std::make_shared<const_value_option>("Distance in mm between the stereo imagers",
                                                                                                  lazy<float>([this]() { return get_stereo_baseline_mm(); })));

        if (advanced_mode && _fw_version >= firmware_version("5.6.3.0"))
        {
            auto depth_scale = std::make_shared<cs_depth_scale_option>(*_hw_monitor);
            auto depth_sensor = As<cs_depth_sensor, cs_sensor>(&depth_ep);
            assert(depth_sensor);

            depth_scale->add_observer([depth_sensor](float val)
                                      {
                                          depth_sensor->set_depth_scale(val);
                                      });

            depth_ep.register_option(RS2_OPTION_DEPTH_UNITS, depth_scale);
        }
        else
            depth_ep.register_option(RS2_OPTION_DEPTH_UNITS, std::make_shared<const_value_option>("Number of meters represented by a single depth unit",
                                                                                                  lazy<float>([]() { return 0.001f; })));

        register_info(RS2_CAMERA_INFO_ADVANCED_MODE, ((advanced_mode) ? "YES" : "NO"));
    }

    std::shared_ptr<cs_sensor> cs_color::create_color_device(std::shared_ptr<context> ctx,
                                                             std::shared_ptr<platform::cs_device> cs_device)
    {
        auto color_ep = std::make_shared<cs_color_sensor>(this, cs_device,
                                                          std::unique_ptr<cs_timestamp_reader>(new cs_timestamp_reader(environment::get_instance().get_time_service())),
                                                          ctx);

        color_ep->register_pixel_format(pf_uyvyc);

        color_ep->try_register_pu(RS2_OPTION_BRIGHTNESS);
        color_ep->try_register_pu(RS2_OPTION_CONTRAST);
        color_ep->try_register_pu(RS2_OPTION_GAIN);
        color_ep->try_register_pu(RS2_OPTION_HUE);
        color_ep->try_register_pu(RS2_OPTION_SATURATION);
        color_ep->try_register_pu(RS2_OPTION_SHARPNESS);
        color_ep->try_register_pu(RS2_OPTION_GAMMA);
        color_ep->try_register_pu(RS2_OPTION_BACKLIGHT_COMPENSATION);

        color_ep->register_option(RS2_OPTION_POWER_LINE_FREQUENCY,
                                  std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_POWER_LINE_FREQUENCY, CS_STREAM_COLOR,
                                                                  std::map<float, std::string>{ { 0.f, "Disabled"},
                                                                                                { 1.f, "50Hz" },
                                                                                                { 2.f, "60Hz" },
                                                                                                { 3.f, "Auto" },
                                                                                                { 4.f, "OutDoor" },}));

        auto exposure_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_EXPOSURE, CS_STREAM_COLOR);
        auto auto_exposure_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_ENABLE_AUTO_EXPOSURE, CS_STREAM_COLOR);
        color_ep->register_option(RS2_OPTION_EXPOSURE, exposure_option);
        color_ep->register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_option);
        color_ep->register_option(RS2_OPTION_EXPOSURE,
                                  std::make_shared<auto_disabling_control>(
                                          exposure_option,
                                          auto_exposure_option));

        auto white_balance_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_WHITE_BALANCE, CS_STREAM_COLOR);
        auto auto_white_balance_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, CS_STREAM_COLOR);
        color_ep->register_option(RS2_OPTION_WHITE_BALANCE, white_balance_option);
        color_ep->register_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, auto_white_balance_option);
        color_ep->register_option(RS2_OPTION_WHITE_BALANCE,
                                  std::make_shared<auto_disabling_control>(
                                          white_balance_option,
                                          auto_white_balance_option));

        return color_ep;
    }

    std::shared_ptr<cs_sensor> cs_mono::create_mono_device(std::shared_ptr<context> ctx,
                                                             std::shared_ptr<platform::cs_device> cs_device)
    {
        auto mono_ep = std::make_shared<cs_mono_sensor>(this, cs_device,
                                                          std::unique_ptr<cs_timestamp_reader>(new cs_timestamp_reader(environment::get_instance().get_time_service())),
                                                          ctx);

        mono_ep->register_pixel_format(pf_raw8);

        /*color_ep->try_register_pu(RS2_OPTION_BRIGHTNESS);
        color_ep->try_register_pu(RS2_OPTION_CONTRAST);
        color_ep->try_register_pu(RS2_OPTION_HUE);
        color_ep->try_register_pu(RS2_OPTION_SATURATION);
        color_ep->try_register_pu(RS2_OPTION_SHARPNESS);*/
        /*color_ep->try_register_pu(RS2_OPTION_GAMMA);

        auto exposure_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_EXPOSURE);
        auto auto_exposure_option = std::make_shared<cs_pu_option>(*color_ep, RS2_OPTION_ENABLE_AUTO_EXPOSURE);
        color_ep->register_option(RS2_OPTION_EXPOSURE, exposure_option);
        color_ep->register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_option);
        color_ep->register_option(RS2_OPTION_EXPOSURE,
                                  std::make_shared<auto_disabling_control>(
                                          exposure_option,
                                          auto_exposure_option));*/

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

        return mono_ep;
    }

    std::shared_ptr<cs_sensor> cs_depth::create_depth_device(std::shared_ptr<context> ctx,
                                                             std::shared_ptr<platform::cs_device> cs_device)
    {
        auto depth_ep = std::make_shared<cs_depth_sensor>(this, cs_device,
                                                          std::unique_ptr<cs_timestamp_reader>(new cs_timestamp_reader(environment::get_instance().get_time_service())),
                                                          ctx);

        depth_ep->try_register_pu(RS2_OPTION_GAIN);
        /*depth_ep->try_register_pu(RS2_OPTION_BRIGHTNESS);
        depth_ep->try_register_pu(RS2_OPTION_CONTRAST);
        depth_ep->try_register_pu(RS2_OPTION_HUE);
        depth_ep->try_register_pu(RS2_OPTION_SATURATION);
        depth_ep->try_register_pu(RS2_OPTION_SHARPNESS);*/
        //depth_ep->try_register_pu(RS2_OPTION_GAMMA);

        auto exposure_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_EXPOSURE, CS_STREAM_DEPTH);
        auto auto_exposure_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_ENABLE_AUTO_EXPOSURE, CS_STREAM_DEPTH);
        depth_ep->register_option(RS2_OPTION_EXPOSURE, exposure_option);
        depth_ep->register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_option);
        depth_ep->register_option(RS2_OPTION_EXPOSURE,
                                  std::make_shared<auto_disabling_control>(
                                          exposure_option,
                                          auto_exposure_option));

        auto emitter_enabled_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_EMITTER_ENABLED, CS_STREAM_DEPTH);
        auto laser_power_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_LASER_POWER, CS_STREAM_DEPTH);
        depth_ep->register_option(RS2_OPTION_EMITTER_ENABLED, emitter_enabled_option);
        depth_ep->register_option(RS2_OPTION_LASER_POWER, laser_power_option);
        depth_ep->register_option(RS2_OPTION_LASER_POWER,
                                  std::make_shared<auto_disabling_control>(
                                          laser_power_option,
                                          emitter_enabled_option,
                                          std::vector<float>{0.f}, 1.f));

        /*auto gain_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_GAIN);
        depth_ep->register_option(RS2_OPTION_GAIN,
                                  std::make_shared<auto_disabling_control>(
                                          gain_option,
                                          auto_exposure_option));

        auto white_balance_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_WHITE_BALANCE);
        auto auto_white_balance_option = std::make_shared<cs_pu_option>(*depth_ep, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
        depth_ep->register_option(RS2_OPTION_WHITE_BALANCE, white_balance_option);
        depth_ep->register_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, auto_white_balance_option);
        depth_ep->register_option(RS2_OPTION_WHITE_BALANCE,
                                  std::make_shared<auto_disabling_control>(
                                          white_balance_option,
                                          auto_white_balance_option));*/
        depth_ep->register_pixel_format(pf_z16);

        return depth_ep;
    }

    bool cs_depth::is_camera_in_advanced_mode() const
    {
        command cmd(ds::UAMG);
        assert(_hw_monitor);
        auto ret = _hw_monitor->send(cmd);
        if (ret.empty())
            throw invalid_value_exception("command result is empty!");

        return (0 != ret.front());
    }

    float cs_depth::get_stereo_baseline_mm() const
    {
        using namespace ds;

        auto table = check_calib<coefficients_table>(*_depth_calib_table_raw);
        return fabs(table->baseline);
    }

   /* void cs_depth::enter_update_state() const
    {
        try {
            LOG_INFO("entering to update state, device disconnect is expected");
            command cmd(ds::DFU);
            cmd.param1 = 1;
            _hw_monitor->send(cmd);
        }
        catch (...) {
            // The set command returns a failure because switching to DFU resets the device while the command is running.
        }
    }

    void cs_depth::update_flash(const std::vector<uint8_t>& image, update_progress_callback_ptr callback, int update_mode)
    {
        throw std::runtime_error("update_flash is not supported by D435e");
    }

    std::vector<uint8_t> cs_depth::backup_flash(update_progress_callback_ptr callback)
    {
        throw std::runtime_error("update_flash is not supported by D435e");
    }

    double cs_depth::get_device_time_ms()
    {
        // TODO: Refactor the following query with an extension.
        if (dynamic_cast<const platform::playback_backend*>(&(get_context()->get_backend())) != nullptr)
        {
            throw not_implemented_exception("device time not supported for backend.");
        }

        if (!_hw_monitor)
            throw wrong_api_call_sequence_exception("_hw_monitor is not initialized yet");

        command cmd(ds::MRD, ds::REGISTER_CLOCK_0, ds::REGISTER_CLOCK_0 + 4);
        auto res = _hw_monitor->send(cmd);

        if (res.size() < sizeof(uint32_t))
        {
            LOG_DEBUG("size(res):" << res.size());
            throw std::runtime_error("Not enough bytes returned from the firmware!");
        }
        uint32_t dt = *(uint32_t*)res.data();
        double ts = dt * TIMESTAMP_USEC_TO_MSEC;
        return ts;
    }*/

    std::vector<uint8_t> cs_color::get_raw_calibration_table(ds::calibration_table_id table_id) const
    {
        command cmd(ds::GETINTCAL, table_id);

        std::vector<uint8_t> temp_color_calib_table_raw = _hw_monitor->send(cmd);
        auto header = reinterpret_cast<const ds::table_header*>(temp_color_calib_table_raw.data());

        temp_color_calib_table_raw.resize(header->table_size + sizeof(ds::table_header));

        return temp_color_calib_table_raw;
    }

    std::vector<uint8_t> cs_depth::get_raw_calibration_table(ds::calibration_table_id table_id) const
    {
        command cmd(ds::GETINTCAL, table_id);

        std::vector<uint8_t> temp_depth_calib_table_raw = _hw_monitor->send(cmd);
        auto header = reinterpret_cast<const ds::table_header*>(temp_depth_calib_table_raw.data());

        temp_depth_calib_table_raw.resize(header->table_size + sizeof(ds::table_header));

        return temp_depth_calib_table_raw;
    }

    std::vector<uint8_t> cs_depth::get_new_calibration_table() const
    {
        //TODO provjeriti da li je treba resizati
        if (_fw_version >= firmware_version("5.11.9.5"))
        {
            command cmd(ds::RECPARAMSGET);
            return _hw_monitor->send(cmd);
        }
        return {};
    }

    ds::d400_caps cs_depth::parse_device_capabilities(const uint16_t pid) const
    {
        using namespace ds;
        std::array<unsigned char,HW_MONITOR_BUFFER_SIZE> gvd_buf;
        _hw_monitor->get_gvd(gvd_buf.size(), gvd_buf.data(), GVD);

        // Opaque retrieval
        d400_caps val{d400_caps::CAP_UNDEFINED};
        if (gvd_buf[active_projector])  // DepthActiveMode
            val |= d400_caps::CAP_ACTIVE_PROJECTOR;
        if (gvd_buf[rgb_sensor])                           // WithRGB
            val |= d400_caps::CAP_RGB_SENSOR;
        if (gvd_buf[imu_sensor])
        {
            val |= d400_caps::CAP_IMU_SENSOR;
            if (hid_bmi_055_pid.end() != hid_bmi_055_pid.find(pid))
                val |= d400_caps::CAP_BMI_055;
            else
            {
                if (hid_bmi_085_pid.end() != hid_bmi_085_pid.find(pid))
                    val |= d400_caps::CAP_BMI_085;
                else
                    LOG_WARNING("The IMU sensor is undefined for PID " << std::hex << pid << std::dec);
            }
        }
        if (0xFF != (gvd_buf[fisheye_sensor_lb] & gvd_buf[fisheye_sensor_hb]))
            val |= d400_caps::CAP_FISHEYE_SENSOR;
        if (0x1 == gvd_buf[depth_sensor_type])
            val |= d400_caps::CAP_ROLLING_SHUTTER;  // e.g. ASRC
        if (0x2 == gvd_buf[depth_sensor_type])
            val |= d400_caps::CAP_GLOBAL_SHUTTER;   // e.g. AWGC

        return val;
    }

    processing_blocks cs_depth::get_cs_depth_recommended_proccesing_blocks() const
    {
        auto res = get_depth_recommended_proccesing_blocks();
        res.push_back(std::make_shared<threshold>());
        res.push_back(std::make_shared<disparity_transform>(true));
        res.push_back(std::make_shared<spatial_filter>());
        res.push_back(std::make_shared<temporal_filter>());
        res.push_back(std::make_shared<hole_filling_filter>());
        res.push_back(std::make_shared<disparity_transform>(false));
        return res;
    }

    std::vector<uint8_t> cs_depth::send_receive_raw_data(const std::vector<uint8_t>& input)
    {
        return _hw_monitor->send(input);
    }

    void cs_depth::create_snapshot(std::shared_ptr<debug_interface>& snapshot) const
    {
        //TODO: Implement
    }

    void cs_depth::enable_recording(std::function<void(const debug_interface&)> record_action)
    {
        //TODO: Implement
    }

    CSMono_camera::CSMono_camera(std::shared_ptr<context> ctx,
                                 const platform::cs_device_info &hwm_device,
                                 const platform::backend_device_group& group,
                                 bool register_device_notifications)
            : device(ctx, group, register_device_notifications),
              cs_mono(ctx, group, register_device_notifications)
    {
        _cs_device = ctx->get_backend().create_cs_device(hwm_device);
        _mono_device_idx = add_sensor(create_mono_device(ctx, _cs_device));

        register_info(RS2_CAMERA_INFO_NAME, hwm_device.info);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, hwm_device.id);
    }

    D435e_camera::D435e_camera(std::shared_ptr<context> ctx,
                               const platform::cs_device_info &hwm_device,
                               const platform::backend_device_group& group,
                               bool register_device_notifications)
            : device(ctx, group, register_device_notifications),
              cs_color(ctx, group, register_device_notifications),
              cs_depth(ctx, group, register_device_notifications),
              cs_advanced_mode_base()
    {
        _cs_device = ctx->get_backend().create_cs_device(hwm_device);

        _depth_device_idx = add_sensor(create_depth_device(ctx, _cs_device));
        _color_device_idx = add_sensor(create_color_device(ctx, _cs_device));

        depth_init(ctx, group);
        color_init(ctx, group);

        environment::get_instance().get_extrinsics_graph().register_extrinsics(*_color_stream, *_depth_stream, _color_extrinsic);

        cs_advanced_mode_init(cs_depth::_hw_monitor, &get_depth_sensor());

        register_info(RS2_CAMERA_INFO_NAME, "FRAMOS D435e"/*hwm_device.info*/);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, "0B07"/*hwm_device.id*/);
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, cs_depth::_fw_version);
        //register_info(RS2_CAMERA_INFO_DEVICE_VERSION, _cs_device->get_device_version());
    }

    std::shared_ptr<matcher> CSMono_camera::create_matcher(const frame_holder& frame) const
    {
        std::vector<stream_interface*> streams = { _fisheye_stream.get()};
        if (frame.frame->supports_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER))
        {
            return matcher_factory::create(RS2_MATCHER_DLR_C, streams);
        }
        return matcher_factory::create(RS2_MATCHER_DEFAULT, streams);
    }

    std::shared_ptr<matcher> D435e_camera::create_matcher(const frame_holder& frame) const
    {
        std::vector<stream_interface*> streams = {_color_stream.get(), _depth_stream.get()/*, _left_ir_stream.get() , _right_ir_stream.get()*/};
        if (frame.frame->supports_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER))
        {
            return matcher_factory::create(RS2_MATCHER_DLR_C, streams);
        }
        return matcher_factory::create(RS2_MATCHER_DEFAULT, streams);
    }

    std::vector<tagged_profile> CSMono_camera::get_profiles_tags() const
    {
        std::vector<tagged_profile> markers;
        markers.push_back({ RS2_STREAM_ANY, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        return markers;
    }

    std::vector<tagged_profile> D435e_camera::get_profiles_tags() const
    {
        std::vector<tagged_profile> markers;
        markers.push_back({ RS2_STREAM_DEPTH, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        markers.push_back({ RS2_STREAM_COLOR, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        return markers;
    }

    void cs_pu_option::set(float value)
    {
        _ep.invoke_powered(
            [this, value](platform::cs_device& dev)
            {
                if (!dev.set_pu(_id, static_cast<int32_t>(value), _stream))
                    throw invalid_value_exception(to_string() << "get_pu(id=" << std::to_string(_id) << ") failed!" << " Last Error: " << strerror(errno));

                _record(*this);
            });
    }

    float cs_pu_option::query() const
    {
        return static_cast<float>(_ep.invoke_powered(
            [this](platform::cs_device& dev)
            {
                int32_t value = 0;
                if (!dev.get_pu(_id, value, _stream))
                    throw invalid_value_exception(to_string() << "get_pu(id=" << std::to_string(_id) << ") failed!" << " Last Error: " << strerror(errno));
                return static_cast<float>(value);
            }));
    }

    option_range cs_pu_option::get_range() const
    {
        auto cs_range = _ep.invoke_powered(
            [this](platform::cs_device& dev)
            {
                return dev.get_pu_range(_id, _stream);
            });

        if (cs_range.min.size() < sizeof(int32_t)) return option_range{0,0,1,0};

        auto min = *(reinterpret_cast<int32_t*>(cs_range.min.data()));
        auto max = *(reinterpret_cast<int32_t*>(cs_range.max.data()));
        auto step = *(reinterpret_cast<int32_t*>(cs_range.step.data()));
        auto def = *(reinterpret_cast<int32_t*>(cs_range.def.data()));
        return option_range{static_cast<float>(min),
                            static_cast<float>(max),
                            static_cast<float>(step),
                            static_cast<float>(def)};
    }

    const char* cs_pu_option::get_description() const
    {
        switch(_id)
        {
            case RS2_OPTION_BACKLIGHT_COMPENSATION: return "Enable / disable backlight compensation";
            case RS2_OPTION_BRIGHTNESS: return "CS image brightness";
            case RS2_OPTION_CONTRAST: return "CS image contrast";
            case RS2_OPTION_EXPOSURE: return "Controls exposure time of color camera. Setting any value will disable auto exposure";
            case RS2_OPTION_GAIN: return "CS image gain";
            case RS2_OPTION_GAMMA: return "CS image gamma setting";
            case RS2_OPTION_HUE: return "CS image hue";
            case RS2_OPTION_SATURATION: return "CS image saturation setting";
            case RS2_OPTION_SHARPNESS: return "CS image sharpness setting";
            case RS2_OPTION_WHITE_BALANCE: return "Controls white balance of color image. Setting any value will disable auto white balance";
            case RS2_OPTION_ENABLE_AUTO_EXPOSURE: return "Enable / disable auto-exposure";
            case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE: return "Enable / disable auto-white-balance";
            case RS2_OPTION_POWER_LINE_FREQUENCY: return "Power Line Frequency";
            case RS2_OPTION_AUTO_EXPOSURE_PRIORITY: return "Limit exposure time when auto-exposure is ON to preserve constant fps rate";
            default: return _ep.get_option_name(_id);
        }
    }

    processing_blocks cs_color_sensor::get_recommended_processing_blocks() const
    {
        return get_color_recommended_proccesing_blocks();
    }

    processing_blocks cs_depth_sensor::get_recommended_processing_blocks() const
    {
        return _owner->get_cs_depth_recommended_proccesing_blocks();
    }

    void cs_depth_sensor::create_snapshot(std::shared_ptr<depth_sensor>& snapshot) const
    {
        snapshot = std::make_shared<depth_sensor_snapshot>(get_depth_scale());
    }

    void cs_depth_sensor::create_snapshot(std::shared_ptr<depth_stereo_sensor>& snapshot) const
    {
        snapshot = std::make_shared<depth_stereo_sensor_snapshot>(get_depth_scale(), get_stereo_baseline_mm());
    }

    void cs_depth_sensor::enable_recording(std::function<void(const depth_sensor&)> recording_function)
    {
        //does not change over time
    }

    void cs_depth_sensor::enable_recording(std::function<void(const depth_stereo_sensor&)> recording_function)
    {
        //does not change over time
    }

    rs2_intrinsics cs_color_sensor::get_intrinsics(const stream_profile& profile) const
    {
        return get_intrinsic_by_resolution(
                *_owner->_color_calib_table_raw,
                ds::calibration_table_id::rgb_calibration_id,
                profile.width, profile.height);
    }

    rs2_intrinsics cs_depth_sensor::get_intrinsics(const stream_profile& profile) const
    {
        rs2_intrinsics result;

        if (ds::try_get_intrinsic_by_resolution_new(*_owner->_new_calib_table_raw,
                                                    profile.width, profile.height, &result))
        {
            return result;
        }
        else
        {
            return get_intrinsic_by_resolution(
                    *_owner->_depth_calib_table_raw,
                    ds::calibration_table_id::coefficients_table_id,
                    profile.width, profile.height);
        }
    }
}