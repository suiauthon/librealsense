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

namespace librealsense
{
    cs_auto_exposure_roi_method::cs_auto_exposure_roi_method(cs_sensor& ep, cs_stream stream)
            :_ep(ep), _stream(stream) {}

    void cs_auto_exposure_roi_method::set(const region_of_interest& roi)
    {
        _ep.invoke_powered(
                [this, roi](platform::cs_device& dev)
                {
                    if (!dev.set_auto_exposure_roi(roi, _stream))
                        throw invalid_value_exception(to_string() << "set_roi failed!" << " Last Error: " << strerror(errno));
                });
    }

    region_of_interest cs_auto_exposure_roi_method::get() const
    {
        region_of_interest roi;

        _ep.invoke_powered(
                [this, roi](platform::cs_device& dev)
                {
                    if (!dev.get_auto_exposure_roi(roi, _stream))
                        throw invalid_value_exception(to_string() << "get_roi failed!" << " Last Error: " << strerror(errno));
                });

        return roi;
    }

    void cs_depth::depth_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group)
    {
        using namespace ds;

        _hw_monitor = std::make_shared<cs_hw_monitor>(get_depth_sensor());

        _depth_extrinsic = std::make_shared<lazy<rs2_extrinsics>>([this]()
                {
                    rs2_extrinsics ext = identity_matrix();
                    auto header = reinterpret_cast<const table_header*>(_depth_calib_table_raw->data());

                    _depth_calib_table_raw->resize(header->table_size + sizeof(table_header));

                    auto table = check_calib<coefficients_table>(*_depth_calib_table_raw);
                    ext.translation[0] = 0.001f * table->baseline; // mm to meters
                    return ext;
                });

        _depth_calib_table_raw = [this]() { return get_raw_calibration_table(coefficients_table_id); };

        //for (int i = 0; i<_depth_calib_table_raw->size(); i++)
        //{
            //printf("%d ", _depth_calib_table_raw->data()[i]);
        //}

        //printf("\n");
        _depth_calib_table_raw->size();
        (*(*_depth_extrinsic)).rotation[0];

        /*for (int i = 0; i<9; i++) printf("%.2f ", (*(*_depth_extrinsic)).rotation[i]);

        printf("\n");

        for (int i = 0; i<3; i++) printf("%.2f ", (*(*_depth_extrinsic)).translation[i]);

        printf("velicina tablice %d\n", _depth_calib_table_raw->size());*/
    }

    std::shared_ptr<cs_sensor> cs_color::create_color_device(std::shared_ptr<context> ctx,
                                                             std::shared_ptr<platform::cs_device> cs_device)
    {
        auto color_ep = std::make_shared<cs_color_sensor>(this, cs_device,
                                                          std::unique_ptr<cs_timestamp_reader>(new cs_timestamp_reader(environment::get_instance().get_time_service())),
                                                          ctx);

        color_ep->register_pixel_format(pf_yuv442);

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

        depth_ep->register_pixel_format(pf_z16);

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
        return depth_ep;
    }

    float cs_depth::get_stereo_baseline_mm() const
    {
        using namespace ds;
        auto table = check_calib<coefficients_table>(*_depth_calib_table_raw);
        return fabs(table->baseline);
    }

    std::vector<uint8_t> cs_depth::get_raw_calibration_table(ds::calibration_table_id table_id) const
    {
        command cmd(ds::GETINTCAL, table_id);
        return _hw_monitor->send(cmd);
    }

    ds::d400_caps cs_depth::parse_device_capabilities() const
    {
        using namespace ds;
        std::array<unsigned char, HW_MONITOR_BUFFER_SIZE> gvd_buf;
        _hw_monitor->get_gvd(gvd_buf.size(), gvd_buf.data(), GVD);

        // Opaque retrieval
        d400_caps val{ d400_caps::CAP_UNDEFINED };
        if (gvd_buf[active_projector])  // DepthActiveMode
            val |= d400_caps::CAP_ACTIVE_PROJECTOR;
        if (gvd_buf[rgb_sensor])                           // WithRGB
            val |= d400_caps::CAP_RGB_SENSOR;
        if (gvd_buf[imu_sensor])
            val |= d400_caps::CAP_IMU_SENSOR;
        if (0xFF != (gvd_buf[fisheye_sensor_lb] & gvd_buf[fisheye_sensor_hb]))
            val |= d400_caps::CAP_FISHEYE_SENSOR;
        if (0x1 == gvd_buf[depth_sensor_type])
            val |= d400_caps::CAP_ROLLING_SHUTTER;  // Standard depth
        if (0x2 == gvd_buf[depth_sensor_type])
            val |= d400_caps::CAP_GLOBAL_SHUTTER;   // Wide depth

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

    cs_camera::cs_camera(std::shared_ptr<context> ctx,
                         const platform::cs_device_info &hwm_device,
                         const platform::backend_device_group& group,
                         bool register_device_notifications)
            : device(ctx, group, register_device_notifications)
    {
        register_info(RS2_CAMERA_INFO_NAME, hwm_device.info);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, hwm_device.id);
        //TODO ovo treba dodati ako se hoce koristiti fw loger, ali tu fali puno stvari od dohvacanja podataka s kamere koje
        //TODO ja u ovom trenutku ne mogu napraviti
        //TODO isto tako bi trebalo registrirati info RS2_CAMERA_INFO_PHYSICAL_PORT
        //register_info(RS2_CAMERA_INFO_DEBUG_OP_CODE, std::to_string(static_cast<int>(fw_cmd::GLD)));
    }

    CSMono_camera::CSMono_camera(std::shared_ptr<context> ctx,
                                 const platform::cs_device_info &hwm_device,
                                 const platform::backend_device_group& group,
                                 bool register_device_notifications)
            : device(ctx, group, register_device_notifications),
              //cs_camera(ctx, hwm_device, group, register_device_notifications),
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
              //cs_camera(ctx, hwm_device, group, register_device_notifications),
              cs_color(ctx, group, register_device_notifications),
              cs_depth(ctx, group, register_device_notifications)
    {
        _cs_device = ctx->get_backend().create_cs_device(hwm_device);

        _color_device_idx = add_sensor(create_color_device(ctx, _cs_device));
        _depth_device_idx = add_sensor(create_depth_device(ctx, _cs_device));

        depth_init(ctx, group);

        register_info(RS2_CAMERA_INFO_NAME, hwm_device.info);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, hwm_device.id);

        auto& depth_ep = get_cs_sensor(_depth_device_idx);

        roi_sensor_interface* roi_sensor;
        if (roi_sensor = dynamic_cast<roi_sensor_interface*>(&depth_ep))
            roi_sensor->set_roi_method(std::make_shared<cs_auto_exposure_roi_method>(depth_ep, CS_STREAM_DEPTH));
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
        std::vector<stream_interface*> streams = {_color_stream.get(), _depth_stream.get()};
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
        markers.push_back({ RS2_STREAM_ANY, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        return markers;
    }

    std::vector<uint8_t> cs_camera::send_receive_raw_data(const std::vector<uint8_t>& input)
    {
        std::vector<uint8_t> a;
        return a;
        //TODO implement
    }

    void cs_camera::create_snapshot(std::shared_ptr<debug_interface>& snapshot) const
    {
        //TODO implement
    }

    void cs_camera::enable_recording(std::function<void(const debug_interface&)> record_action)
    {
        //TODO implement
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

    rs2_intrinsics cs_depth_sensor::get_intrinsics(const stream_profile& profile) const
    {
        return get_intrinsic_by_resolution(
                *_owner->_depth_calib_table_raw,
                ds::calibration_table_id::coefficients_table_id,
                profile.width, profile.height);
    }
}