//
// Created by marko on 09.03.19..
//

#include "cs.h"
#include <iomanip>

namespace librealsense
{
    std::shared_ptr<device_interface> cs_info::create(std::shared_ptr<context> ctx,
                                                      bool register_device_notifications) const
    {
        return std::make_shared<cs_camera>(ctx, _hwm, this->get_device_data(),
                                           register_device_notifications);
    }

    std::vector <std::shared_ptr<device_info>> cs_info::pick_cs_devices(
        std::shared_ptr <context> ctx,
        std::vector <platform::usb_device_info> &usb)
    {
        std::vector<platform::usb_device_info> chosen;
        std::vector<std::shared_ptr<device_info>> results;

        auto correct_pid = filter_by_product(usb, { UCC2592C_PID });
        auto group_devices = group_devices_by_unique_id(correct_pid);
        for (auto& group : group_devices)
        {
            if (!group.empty() && vid_present(group, VID_SMARTEK_CAMERA))
            {
                auto color = get_vid(group, VID_SMARTEK_CAMERA);

                auto info = std::make_shared<cs_info>(ctx, color);
                chosen.push_back(color);
                results.push_back(info);
            }
            else
            {
                LOG_WARNING("CS group_devices is empty.");
            }
        }
        trim_device_list(usb, chosen);

        return results;
    }

    cs_camera::cs_camera(std::shared_ptr<context> ctx,
                         const platform::usb_device_info &hwm_device,
                         const platform::backend_device_group& group,
                         bool register_device_notifications)
        : device(ctx, group, register_device_notifications)
    {
        auto color_ep = std::make_shared<cs_sensor>(this, ctx->get_backend().create_usb_device(hwm_device),
                                                    std::unique_ptr<cs_timestamp_reader>(new cs_timestamp_reader()),
                                                    ctx);

        add_sensor(color_ep);

        register_info(RS2_CAMERA_INFO_NAME, "CS Camera");
        std::string pid_str(to_string() << std::setfill('0') << std::setw(4) << std::hex << hwm_device.pid);
        std::transform(pid_str.begin(), pid_str.end(), pid_str.begin(), ::toupper);

        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.unique_id);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, pid_str);

        //TODO
        //napraviti ove registracije opcija

        /*
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, pid_str);

        color_ep->register_pixel_format(pf_yuy2);
        color_ep->register_pixel_format(pf_yuyv);

        color_ep->try_register_pu(RS2_OPTION_BACKLIGHT_COMPENSATION);
        color_ep->try_register_pu(RS2_OPTION_BRIGHTNESS);
        color_ep->try_register_pu(RS2_OPTION_CONTRAST);
        color_ep->try_register_pu(RS2_OPTION_EXPOSURE);
        color_ep->try_register_pu(RS2_OPTION_GAMMA);
        color_ep->try_register_pu(RS2_OPTION_HUE);
        color_ep->try_register_pu(RS2_OPTION_SATURATION);
        color_ep->try_register_pu(RS2_OPTION_SHARPNESS);
        color_ep->try_register_pu(RS2_OPTION_WHITE_BALANCE);
        color_ep->try_register_pu(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
        color_ep->try_register_pu(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);





        color_ep->register_pixel_format(pf_yuy2);
        color_ep->register_pixel_format(pf_yuyv);

        color_ep->register_pu(RS2_OPTION_BACKLIGHT_COMPENSATION);
        color_ep->register_pu(RS2_OPTION_BRIGHTNESS);
        color_ep->register_pu(RS2_OPTION_CONTRAST);
        color_ep->register_pu(RS2_OPTION_GAIN);
        color_ep->register_pu(RS2_OPTION_GAMMA);
        color_ep->register_pu(RS2_OPTION_HUE);
        color_ep->register_pu(RS2_OPTION_SATURATION);
        color_ep->register_pu(RS2_OPTION_SHARPNESS);

        auto white_balance_option = std::make_shared<uvc_pu_option>(*color_ep, RS2_OPTION_WHITE_BALANCE);
        auto auto_white_balance_option = std::make_shared<uvc_pu_option>(*color_ep, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
        color_ep->register_option(RS2_OPTION_WHITE_BALANCE, white_balance_option);
        color_ep->register_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, auto_white_balance_option);
        color_ep->register_option(RS2_OPTION_WHITE_BALANCE,
                                  std::make_shared<auto_disabling_control>(
                                          white_balance_option,
                                          auto_white_balance_option));

        auto exposure_option = std::make_shared<uvc_pu_option>(*color_ep, RS2_OPTION_EXPOSURE);
        auto auto_exposure_option = std::make_shared<uvc_pu_option>(*color_ep, RS2_OPTION_ENABLE_AUTO_EXPOSURE);
        color_ep->register_option(RS2_OPTION_EXPOSURE, exposure_option);
        color_ep->register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_option);
        color_ep->register_option(RS2_OPTION_EXPOSURE,
                                  std::make_shared<auto_disabling_control>(
                                          exposure_option,
                                          auto_exposure_option));

        auto md_offset = offsetof(metadata_raw, mode);
        color_ep->register_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP, make_uvc_header_parser(&platform::uvc_header::timestamp,
                                                                                               [](rs2_metadata_type param) { return static_cast<rs2_metadata_type>(param * TIMESTAMP_10NSEC_TO_MSEC); }));
        color_ep->register_metadata(RS2_FRAME_METADATA_FRAME_COUNTER,   make_sr300_attribute_parser(&md_sr300_rgb::frame_counter, md_offset));
        color_ep->register_metadata(RS2_FRAME_METADATA_ACTUAL_FPS,      make_sr300_attribute_parser(&md_sr300_rgb::actual_fps, md_offset));
        color_ep->register_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP,make_sr300_attribute_parser(&md_sr300_rgb::frame_latency, md_offset));
        color_ep->register_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE, make_sr300_attribute_parser(&md_sr300_rgb::actual_exposure, md_offset, [](rs2_metadata_type param) { return param*100; }));
        color_ep->register_metadata(RS2_FRAME_METADATA_AUTO_EXPOSURE,   make_sr300_attribute_parser(&md_sr300_rgb::auto_exp_mode, md_offset, [](rs2_metadata_type param) { return (param !=1); }));
        color_ep->register_metadata(RS2_FRAME_METADATA_GAIN_LEVEL,      make_sr300_attribute_parser(&md_sr300_rgb::gain, md_offset));
        color_ep->register_metadata(RS2_FRAME_METADATA_WHITE_BALANCE,   make_sr300_attribute_parser(&md_sr300_rgb::color_temperature, md_offset));*/

    }

    cs_timestamp_reader::cs_timestamp_reader(void)
    {
        reset();
    }

    void cs_timestamp_reader::reset()
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        /*for (auto i = 0; i < pins; ++i)
        {
            counter[i] = 0;
        }*/
    }

    rs2_time_t cs_timestamp_reader::get_frame_timestamp(const request_mapping& mode, const platform::frame_object& fo)
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        //return _ts->get_time();
        return 0;
    }

    unsigned long long cs_timestamp_reader::get_frame_counter(const request_mapping & mode, const platform::frame_object& fo) const
    {
        /*std::lock_guard<std::recursive_mutex> lock(_mtx);
        auto pin_index = 0;
        if (mode.pf->fourcc == 0x5a313620) // Z16
            pin_index = 1;

        return ++counter[pin_index];*/
        return 0;
    }

    rs2_timestamp_domain cs_timestamp_reader::get_frame_timestamp_domain(const request_mapping & mode, const platform::frame_object& fo) const
    {
        return RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME;
    }

    void cs_sensor::open(const stream_profiles& requests)
    {

    }

    void cs_sensor::close()
    {

    }

    void cs_sensor::start(frame_callback_ptr callback)
    {

    }

    void cs_sensor::stop()
    {

    }

}