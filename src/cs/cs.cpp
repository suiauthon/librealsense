//
// Created by marko on 09.03.19..
//

#include "cs.h"
#include <iomanip>
#include <stream.h>
#include "environment.h"

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
        std::vector <platform::cs_device_info> &cs)
    {
        std::vector<std::shared_ptr<device_info>> results;

        for (auto& group : cs)
        {
            auto info = std::make_shared<cs_info>(ctx, group);
            results.push_back(info);
        }

        return results;
    }

    std::vector<platform::cs_device_info> cs_info::query_cs_devices()
    {
        std::vector<platform::cs_device_info> results;
        std::string string_node;

        auto smcs_api = smcs::GetCameraAPI();
        smcs_api->FindAllDevices(1);
        auto devices = smcs_api->GetAllDevices();

        for (int i = 0; i < devices.size(); i++)
        {
            auto info = platform::cs_device_info();
            info.serial = devices[i]->GetSerialNumber();
            info.id = devices[i]->GetModelName();
            info.info = devices[i]->GetManufacturerSpecificInfo();

            results.push_back(info);
        }

        return results;
    }

    cs_camera::cs_camera(std::shared_ptr<context> ctx,
                         const platform::cs_device_info &hwm_device,
                         const platform::backend_device_group& group,
                         bool register_device_notifications)
        : device(ctx, group, register_device_notifications)
    {
        auto color_ep = std::make_shared<cs_sensor>(this, std::make_shared<platform::cs_device>(hwm_device),
                                                    std::unique_ptr<cs_timestamp_reader>(new cs_timestamp_reader(environment::get_instance().get_time_service())),
                                                    ctx);

        add_sensor(color_ep);

        register_info(RS2_CAMERA_INFO_NAME, hwm_device.info);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, hwm_device.id);

        //std::string pid_str(to_string() << std::setfill('0') << std::setw(4) << std::hex << hwm_device.pid);
        //std::transform(pid_str.begin(), pid_str.end(), pid_str.begin(), ::toupper);

        //register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.unique_id);
        //register_info(RS2_CAMERA_INFO_PRODUCT_ID, pid_str);

        color_ep->register_pixel_format(pf_yuy2);
        color_ep->register_pixel_format(pf_yuyv);
        color_ep->register_pixel_format(pf_raw8);

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

    cs_timestamp_reader::cs_timestamp_reader(std::shared_ptr<platform::time_service> ts):
            _ts(ts)
    {
        reset();
    }

    void cs_timestamp_reader::reset()
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        counter = 0;
    }

    rs2_time_t cs_timestamp_reader::get_frame_timestamp(const request_mapping& mode, const platform::frame_object& fo)
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        return _ts->get_time();
    }

    unsigned long long cs_timestamp_reader::get_frame_counter(const request_mapping & mode, const platform::frame_object& fo) const
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        return ++counter;
        return 0;
    }

    rs2_timestamp_domain cs_timestamp_reader::get_frame_timestamp_domain(const request_mapping & mode, const platform::frame_object& fo) const
    {
        return RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME;
    }

    void cs_sensor::open(const stream_profiles& requests)
    {
        printf("OPEN\n");
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("open(...) failed. CS device is streaming!");
        else if (_is_opened)
            throw wrong_api_call_sequence_exception("open(...) failed. CS device is already opened!");

        auto on = std::unique_ptr<power>(new power(std::dynamic_pointer_cast<cs_sensor>(shared_from_this())));

        _source.init(_metadata_parsers);
        _source.set_sensor(this->shared_from_this());
        auto mapping = resolve_requests(requests);

        auto timestamp_reader = _timestamp_reader.get();

        std::vector<platform::stream_profile> commited;

        for (auto&& mode : mapping)
        {
            try
            {
                unsigned long long last_frame_number = 0;
                rs2_time_t last_timestamp = 0;
                _device->probe_and_commit(mode.profile,
                                          [this, mode, timestamp_reader, requests, last_frame_number, last_timestamp]
                                          (platform::stream_profile p, platform::frame_object f,
                                           std::function<void()> continuation) mutable
                                          {
                                              auto system_time = environment::get_instance().get_time_service()->get_time();
                                              if (!this->is_streaming())
                                              {
                                                  LOG_WARNING("Frame received with streaming inactive,"
                                                                      << librealsense::get_string(mode.unpacker->outputs.front().stream_desc.type)
                                                                      << mode.unpacker->outputs.front().stream_desc.index
                                                                      << ", Arrived," << std::fixed << f.backend_time << " " << system_time);
                                                  return;
                                              }
                                              //frame_continuation release_and_enqueue(continuation, f.pixels);

                                              // Ignore any frames which appear corrupted or invalid
                                              // Determine the timestamp for this frame
                                              auto timestamp = timestamp_reader->get_frame_timestamp(mode, f);
                                              printf("Timestamp callback: %lf\n", timestamp);
                                              auto timestamp_domain = timestamp_reader->get_frame_timestamp_domain(mode, f);
                                              printf("Timestamp domain callback: %d\n", timestamp_domain);
                                              auto frame_counter = timestamp_reader->get_frame_counter(mode, f);

                                              auto requires_processing = mode.requires_processing();

                                              std::vector<byte *> dest;
                                              std::vector<frame_holder> refs;

                                              auto&& unpacker = *mode.unpacker;
                                              for (auto&& output : unpacker.outputs)
                                              {
                                                  LOG_DEBUG("FrameAccepted," << librealsense::get_string(output.stream_desc.type)
                                                                             << ",Counter," << std::dec << frame_counter
                                                                             << ",Index," << output.stream_desc.index
                                                                             << ",BackEndTS," << std::fixed << f.backend_time
                                                                             << ",SystemTime," << std::fixed << system_time
                                                                             <<" ,diff_ts[Sys-BE],"<< system_time- f.backend_time
                                                                             << ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain)
                                                                             <<",last_frame_number,"<< last_frame_number<<",last_timestamp,"<< last_timestamp);

                                                  std::shared_ptr<stream_profile_interface> request = nullptr;
                                                  for (auto&& original_prof : mode.original_requests)
                                                  {
                                                      if (original_prof->get_format() == output.format &&
                                                          original_prof->get_stream_type() == output.stream_desc.type &&
                                                          original_prof->get_stream_index() == output.stream_desc.index)
                                                      {
                                                          request = original_prof;
                                                      }
                                                  }

                                                  auto bpp = get_image_bpp(output.format);
                                                  frame_additional_data additional_data(timestamp,
                                                                                        frame_counter,
                                                                                        system_time,
                                                                                        static_cast<uint8_t>(f.metadata_size),
                                                                                        (const uint8_t*)f.metadata,
                                                                                        f.backend_time,
                                                                                        last_timestamp,
                                                                                        last_frame_number,
                                                                                        false);

                                                  last_frame_number = frame_counter;
                                                  last_timestamp = timestamp;

                                                  auto res = output.stream_resolution({ mode.profile.width, mode.profile.height });
                                                  auto width = res.width;
                                                  auto height = res.height;

                                                  frame_holder frame = _source.alloc_frame(stream_to_frame_types(output.stream_desc.type), width * height * bpp / 8, additional_data, requires_processing);
                                                  if (frame.frame)
                                                  {
                                                      auto video = (video_frame*)frame.frame;
                                                      video->assign(width, height, width * bpp / 8, bpp);
                                                      video->set_timestamp_domain(timestamp_domain);
                                                      dest.push_back(const_cast<byte*>(video->get_frame_data()));
                                                      frame->set_stream(request);
                                                      refs.push_back(std::move(frame));
                                                  }
                                                  else
                                                  {
                                                      LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
                                                      return;
                                                  }

                                              }
                                              // Unpack the frame
                                              if (requires_processing && (dest.size() > 0))
                                              {
                                                  unpacker.unpack(dest.data(), reinterpret_cast<const byte *>(f.pixels), mode.profile.width, mode.profile.height);
                                              }

                                              // If any frame callbacks were specified, dispatch them now
                                              for (auto&& pref : refs)
                                              {
                                                  /*if (!requires_processing)
                                                  {
                                                      pref->attach_continuation(std::move(release_and_enqueue));
                                                  }*/

                                                  if (_on_before_frame_callback)
                                                  {
                                                      printf("Frame data: %d\n",pref.frame->get_frame_data()[0]);
                                                      auto callback = _source.begin_callback();
                                                      auto stream_type = pref->get_stream()->get_stream_type();
                                                      _on_before_frame_callback(stream_type, pref, std::move(callback));
                                                  }

                                                  if (pref->get_stream().get())
                                                  {
                                                      printf("Frame data: %d\n",pref.frame->get_frame_data()[0]);
                                                      _source.invoke_callback(std::move(pref));
                                                  }
                                              }

                                          });

            }
            catch(...)
            {
                for (auto&& commited_profile : commited)
                {
                    _device->close(commited_profile);
                }
                throw;
            }
            commited.push_back(mode.profile);
        }

        _internal_config = commited;

        if (_on_open)
            _on_open(_internal_config);

        _power = move(on);
        _is_opened = true;

        try {
            _device->stream_on([&](const notification& n)
                               {
                                   _notifications_processor->raise_notification(n);
                               });
        }
        catch (...)
        {
            for (auto& profile : _internal_config)
            {
                try {
                    _device->close(profile);
                }
                catch (...) {}
            }
            reset_streaming();
            _power.reset();
            _is_opened = false;
            throw;
        }
        set_active_streams(requests);
    }

    rs2_extension cs_sensor::stream_to_frame_types(rs2_stream stream)
    {
        // TODO: explicitly return video_frame for relevant streams and default to an error?
        switch (stream)
        {
            case RS2_STREAM_DEPTH:  return RS2_EXTENSION_DEPTH_FRAME;
            default:                return RS2_EXTENSION_VIDEO_FRAME;
        }
    }

    void cs_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("close() failed. UVC device is streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("close() failed. UVC device was not opened!");

        for (auto& profile : _internal_config)
        {
            _device->close(profile);
        }
        reset_streaming();
        _power.reset();
        _is_opened = false;
        set_active_streams({});
    }

    void cs_sensor::start(frame_callback_ptr callback)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. UVC device is already streaming!");
        else if(!_is_opened)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. UVC device was not opened!");

        _source.set_callback(callback);
        _is_streaming = true;
        raise_on_before_streaming_changes(true); //Required to be just before actual start allow recording to work
        _device->start_callbacks();
    }

    void cs_sensor::stop()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (!_is_streaming)
            throw wrong_api_call_sequence_exception("stop_streaming() failed. UVC device is not streaming!");

        _is_streaming = false;
        _device->stop_callbacks();
        raise_on_before_streaming_changes(false);
    }

    void cs_sensor::acquire_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(1) == 0)
        {
            _device->set_power_state(platform::D0);
        }
    }

    void cs_sensor::release_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(-1) == 1)
        {
            _device->set_power_state(platform::D3);
        }
    }

    void cs_sensor::reset_streaming()
    {
        _source.flush();
        _source.reset();
        _timestamp_reader->reset();
    }

    namespace platform
    {
        std::vector<stream_profile> cs_device::get_profiles()
        {
            std::vector<stream_profile> all_stream_profiles;
            stream_profile profile;
            INT64 int64Value;
            std::string node_value;

            if (this->connect())
            {
                if (_connected_device->GetIntegerNodeValue("Width", int64Value)) {
                    profile.width = (uint32_t)int64Value;
                }
                if (_connected_device->GetIntegerNodeValue("Height", int64Value)) {
                    profile.height = (uint32_t)int64Value;
                }
                if (_connected_device->GetStringNodeValue("PixelFormat", node_value)) {
                    profile.format = 'GREY';
                }
                if (_connected_device->GetIntegerNodeValue("FPS", int64Value)) {
                    profile.fps = (uint32_t)int64Value;
                }
                else profile.fps = 50;

                all_stream_profiles.push_back(profile);

                this->disconnect();
            }


            return all_stream_profiles;
        }

        void cs_device::close(stream_profile profile)
        {
            if(_is_capturing)
            {
                stop_data_capture();
            }

            if (_callback)
            {
                _callback = nullptr;
            }

        }

        void cs_device::stream_on(std::function<void(const notification& n)> error_handler)
        {
            if(!_is_capturing)
            {
                _error_handler = error_handler;

                start_data_capture();

                _is_capturing = true;
                _thread = std::unique_ptr<std::thread>(new std::thread([this](){ capture_loop(); }));
            }
        }

        uint32_t cs_device::get_rs2_format(std::string format)
        {

        }

        void cs_device::set_power_state(power_state state)
        {
            std::lock_guard<std::mutex> lock(_power_lock);

            if (state == D0 && _power_state == D3)
            {
                _smcs_api = smcs::GetCameraAPI();
                _power_state = D0;
            }
            if (state == D3 && _power_state == D0)
            {
                _power_state = D3;
            }
        }

        bool cs_device::connect(void)
        {
            std::lock_guard<std::mutex> lock(_power_lock);

            std::string serial;

            if (_power_state == D0 && !_is_connected)
            {
                auto devices = _smcs_api->GetAllDevices();

                for (int i = 0; i < devices.size(); i++)
                {
                    serial = devices[i]->GetSerialNumber();
                    if (!serial.compare(_device_info.serial))
                    {
                        _connected_device = devices[i];
                    }
                }

                if (_connected_device != NULL && _connected_device->Connect())
                {
                    _is_connected = true;
                }
            }

            return _is_connected;
        }

        void cs_device::disconnect(void)
        {
            std::lock_guard<std::mutex> lock(_power_lock);
            if (_power_state == D0 && _is_connected)
            {
                if (_connected_device != NULL)
                {
                    _connected_device->Disconnect();
                    _connected_device = NULL;
                    _is_connected = false;
                }
            }
        }

        void cs_device::start_callbacks()
        {
            _is_started = true;
        }

        void cs_device::stop_callbacks()
        {
            _is_started = false;
        }

        void cs_device::stop_data_capture()
        {
            _is_capturing = false;
            _is_started = false;

            if (_is_connected)
            {
                _connected_device->CommandNodeExecute("AcquisitionStop");
                _connected_device->SetIntegerNodeValue("TLParamsLocked", 0);
                this->disconnect();
            }

            _thread->join();
            _thread.reset();
        }

        void cs_device::start_data_capture()
        {
            if (this->connect())
            {
                // disable trigger mode
                _connected_device->SetStringNodeValue("TriggerMode", "Off");
                // set continuous acquisition mode
                _connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");
                // start acquisition
                _connected_device->SetIntegerNodeValue("TLParamsLocked", 1);
                _connected_device->CommandNodeExecute("AcquisitionStart");
            }
        }

        void cs_device::capture_loop()
        {
            try
            {
                while(_is_capturing)
                {
                    poll();
                }
            }
            catch (const std::exception& ex)
            {
                LOG_ERROR(ex.what());

                librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, ex.what()};

                _error_handler(n);
            }
        }

        void cs_device::poll()
        {
            smcs::IImageInfo image_info_;

            UINT32 src_pixel_type;
            UINT32 src_width, src_height;

            if (_connected_device.IsValid() && _connected_device->IsConnected()) {
                if (!_connected_device->IsBufferEmpty()) {
                    _connected_device->GetImageInfo(&image_info_);

                    auto image_id = image_info_->GetImageID();
                    auto timestamp = image_info_->GetCameraTimestamp() / 1000000.0;

                    auto im = smcs::IImageBitmapInterface(image_info_).GetRawData();

                    smcs::IImageBitmapInterface(image_info_).GetPixelType(src_pixel_type);
                    smcs::IImageBitmapInterface(image_info_).GetSize(src_width, src_height);
                    smcs::IImageBitmapInterface(image_info_).GetPixelType(src_pixel_type);

                    auto c = GvspGetBitsPerPixel((GVSP_PIXEL_TYPES)src_pixel_type) / 8;

                    frame_object fo {src_width*src_height*c, 0, im, NULL, timestamp};

                    _callback(_profile, fo, NULL);

                    _connected_device->PopImage(image_info_);
                    _connected_device->ClearImageBuffer();
                }
            }
        }

        void cs_device::probe_and_commit(stream_profile profile, frame_callback callback)
        {
            if(!_is_capturing && !_callback)
            {
                set_format(profile);

                _profile =  profile;
                _callback = callback;
            }
            else
            {
                throw wrong_api_call_sequence_exception("Device already streaming!");
            }
        }

        void cs_device::set_format(stream_profile profile)
        {
            //TODO
            //tu se odabire profil na kameri
        }
    }

}