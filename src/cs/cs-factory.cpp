//
// Created by marko on 21.05.19..
//

#include "cs/cs-factory.h"
#include "cs/cs-device.h"

namespace librealsense {

    cs_stream_id cs_stream_to_id(cs_stream stream)
    {
        switch(stream)
        {
            case CS_STREAM_DEPTH: return CS_STREAM_ID_DEPTH;
            case CS_STREAM_COLOR: return CS_STREAM_ID_COLOR;
            case CS_STREAM_MONO: return CS_STREAM_ID_MONO;
            default: return CS_STREAM_ID_MONO;
        }
    };

    std::shared_ptr <device_interface> cs_info::create(std::shared_ptr <context> ctx,
                                                       bool register_device_notifications) const {
        switch (get_camera_model(_hwm.id)) {
            case CS_UCC2592C:
                return std::make_shared<CSMono_camera>(ctx, _hwm, this->get_device_data(),
                                                       register_device_notifications);
            case CS_UCC1932C:
                return std::make_shared<CSMono_camera>(ctx, _hwm, this->get_device_data(),
                                                       register_device_notifications);
            case CS_D435E:
                return std::make_shared<D435e_camera>(ctx, _hwm, this->get_device_data(),
                                                      register_device_notifications);
            default:
                throw std::runtime_error(to_string() << "Unsupported CS model! 0x"
                                                     << std::hex << std::setw(4) << std::setfill('0') << _hwm.id);
        }
    }

    cs_camera_model cs_info::get_camera_model(std::string pid) {
        if (pid.compare("UCC2592C") == 0) return CS_UCC2592C;
        else if (pid.compare("UCC1932Cg.") == 0) return CS_UCC1932C;
        else if (pid.compare("D435e") == 0) return CS_D435E;
        else return CS_UNDEFINED;
    }

    bool cs_info::is_timestamp_supported(std::string pid) {
        switch (get_camera_model(pid)) {
            case CS_UCC2592C:
                return false;
            case CS_UCC1932C:
                return true;
            default:
                return false;
        }
    }

    std::vector <std::shared_ptr<device_info>> cs_info::pick_cs_devices(
            std::shared_ptr <context> ctx,
            std::vector <platform::cs_device_info> &cs) {
        std::vector <std::shared_ptr<device_info>> results;

        for (auto &group : cs) {
            auto info = std::make_shared<cs_info>(ctx, group);
            results.push_back(info);
        }

        return results;
    }

    std::vector <platform::cs_device_info> cs_info::query_cs_devices() {
        std::vector <platform::cs_device_info> results;
        std::string string_node;

        auto smcs_api = smcs::GetCameraAPI();
        //printf("Trazim\n");
        smcs_api->FindAllDevices(0.15);
        //printf("Nasao\n");
        auto devices = smcs_api->GetAllDevices();

        for (int i = 0; i < devices.size(); i++) {
            //printf("Broj uredaja na kompu %d\n", devices.size());
            if (devices[i]->IsOnNetwork()) {
                //printf("Uredaj je na kompu\n");
                auto info = platform::cs_device_info();
                info.serial = devices[i]->GetSerialNumber();
                info.id = devices[i]->GetModelName();
                //printf("id %s\n", info.id.c_str());
                info.info = devices[i]->GetManufacturerSpecificInfo();


                /*printf("%s\n", devices[i]->GetManufacturerName().c_str());
                printf("%s\n", devices[i]->GetManufacturerSpecificInfo().c_str());
                printf("%s\n", devices[i]->GetSerialNumber().c_str());
                printf("%s\n", devices[i]->GetModelName().c_str());
                printf("%d\n", devices[i]->GetDeviceType()); //dodati da bude tamo ikonica da je usb 3 ili gev
                printf("%d\n", devices[i]->GetGateway());
                printf("%d\n", devices[i]->GetIpAddress());
                printf("%s\n", devices[i]->GetDeviceVersion().c_str());
                printf("%d\n", devices[i]->GetMacAddress());
                printf("%d\n", devices[i]->GetSubnetMask());
                printf("%d\n", devices[i]->GetVersion());*/

                results.push_back(info);
            }
        }

        return results;
    }

    void cs_sensor::open(const stream_profiles& requests)
    {
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
                                              frame_continuation release_and_enqueue(continuation, f.pixels);

                                              // Ignore any frames which appear corrupted or invalid
                                              // Determine the timestamp for this frame
                                              auto timestamp = timestamp_reader->get_frame_timestamp(mode, f);
                                              //printf("Timestamp callback: %lf\n", timestamp);
                                              auto timestamp_domain = timestamp_reader->get_frame_timestamp_domain(mode, f);
                                              //printf("Timestamp domain callback: %d\n", timestamp_domain);
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
                                                  if (!requires_processing)
                                                  {
                                                      pref->attach_continuation(std::move(release_and_enqueue));
                                                  }

                                                  if (_on_before_frame_callback)
                                                  {
                                                      //printf("Frame data: %d\n",pref.frame->get_frame_data()[0]);
                                                      auto callback = _source.begin_callback();
                                                      auto stream_type = pref->get_stream()->get_stream_type();
                                                      _on_before_frame_callback(stream_type, pref, std::move(callback));
                                                  }
                                                  if (pref->get_stream().get())
                                                  {
                                                      //printf("Frame data: %d\n",pref.frame->get_frame_data()[0]);
                                                      _source.invoke_callback(std::move(pref));
                                                  }
                                              }

                                          }, _cs_stream_id);

            }
            catch(...)
            {
                for (auto&& commited_profile : commited)
                {
                    _device->close(commited_profile, _cs_stream_id);
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
                               }, _cs_stream_id);
        }
        catch (...)
        {
            for (auto& profile : _internal_config)
            {
                try {
                    _device->close(profile, _cs_stream_id);
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
            throw wrong_api_call_sequence_exception("close() failed. CS device is streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("close() failed. CS device was not opened!");

        for (auto& profile : _internal_config)
        {
            _device->close(profile, _cs_stream_id);
        }
        reset_streaming();
        _power.reset();
        _is_opened = false;
        set_active_streams({});
    }

    void cs_sensor::start(frame_callback_ptr callback)
    {
        //printf("Cs sensor START\n");
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. CS device is already streaming!");
        else if(!_is_opened)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. CS device was not opened!");

        _source.set_callback(callback);
        _is_streaming = true;
        raise_on_before_streaming_changes(true); //Required to be just before actual start allow recording to work
    }

    void cs_sensor::stop()
    {
        //printf("Cs sensor STOP\n");
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (!_is_streaming)
            throw wrong_api_call_sequence_exception("stop_streaming() failed. CS device is not streaming!");

        _is_streaming = false;
        raise_on_before_streaming_changes(false);
    }

    void cs_sensor::acquire_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(1) == 0)
        {
            if (_device->set_power_state(platform::D0) != platform::D0)
                throw wrong_api_call_sequence_exception("set power state(...) failed. CS device cannot be turned on!");
        }
    }

    void cs_sensor::release_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(-1) == 1)
        {
            if (_device->set_power_state(platform::D3) != platform::D3)
                throw wrong_api_call_sequence_exception("set power state(...) failed. CS device cannot be turned off!");
        }
    }

    void cs_sensor::reset_streaming()
    {
        _source.flush();
        _source.reset();
        _timestamp_reader->reset();
    }

    void cs_sensor::register_pu(rs2_option id)
    {
        register_option(id, std::make_shared<cs_pu_option>(*this, id, _cs_stream));
    }

    void cs_sensor::try_register_pu(rs2_option id)
    {
        try
        {
            auto opt = std::make_shared<cs_pu_option>(*this, id, _cs_stream);
            auto range = opt->get_range();
            if (range.max <= range.min || range.step <= 0 || range.def < range.min || range.def > range.max) return;

            auto val = opt->query();
            if (val < range.min || val > range.max) return;
            opt->set(val);

            register_option(id, opt);
        }
        catch (...)
        {
            LOG_WARNING("Exception was thrown when inspecting properties of a sensor");
        }
    }

    namespace platform
    {
        std::vector<stream_profile> cs_device::get_profiles()
        {
            std::vector<stream_profile> all_stream_profiles;
            stream_profile profile;
            INT64 int64Value;
            smcs::StringList node_value_list;
            bool is_successful = true;

            for (int i = 0; i < _number_of_streams; i++)
            {
                is_successful = is_successful & _connected_device->GetIntegerNodeValue("SensorWidth", int64Value);
                profile.width = (uint32_t)int64Value;

                is_successful = is_successful & _connected_device->GetIntegerNodeValue("SensorHeight", int64Value);
                profile.height = (uint32_t)int64Value;

                is_successful = is_successful & _connected_device->GetEnumNodeValuesList("PixelFormat", node_value_list);

                if (_connected_device->GetIntegerNodeValue("FPS", int64Value)) {
                    profile.fps = (uint32_t)int64Value;
                }
                else profile.fps = 30;

                if (is_successful)
                {
                    profile.format = cs_pixel_format_to_native_pixel_format(node_value_list[i]);
                    all_stream_profiles.push_back(profile);
                }
            }

            return all_stream_profiles;
        }

        uint32_t cs_device::cs_pixel_format_to_native_pixel_format(std::string cs_format)
        {
            uint32_t npf;
            if (cs_format.compare("YUV422Packed") == 0)
                npf = 'YUV ';
            else if (cs_format.compare("Mono16") == 0)
                npf = 'Z16 ';
            else throw wrong_api_call_sequence_exception("Unsuported image format!");

            return npf;
        }


        bool cs_device::get_pu(rs2_option opt, int32_t& value, cs_stream stream)
        {
            return get_cs_param_value(opt, value, stream);
        }

        bool cs_device::set_pu(rs2_option opt, int32_t value, cs_stream stream)
        {
            return set_cs_param(opt, value, stream);
        }

        bool cs_device::get_auto_exposure_roi(region_of_interest roi, cs_stream stream)
        {
            bool status;
            INT64 value;

            //printf("Getam roi\n");

            switch(stream)
            {
                case CS_STREAM_DEPTH:
                    status = _connected_device->GetIntegerNodeValue("STR_ExposureAutoROITop", value);
                    roi.min_y = value;

                    status = status && _connected_device->GetIntegerNodeValue("STR_ExposureAutoROIBottom", value);
                    roi.max_y = value;

                    status = status && _connected_device->GetIntegerNodeValue("STR_ExposureAutoROILeft", value);
                    roi.min_x = value;

                    status = status && _connected_device->GetIntegerNodeValue("STR_ExposureAutoROIRight", value);
                    roi.max_x = value;

                    return status;
                case CS_STREAM_COLOR:
                    return false;
            }
        }

        bool cs_device::set_auto_exposure_roi(const region_of_interest &roi, cs_stream stream)
        {
            bool status;
            INT64 value;

            //printf("setam roi\n");

            switch(stream)
            {
                case CS_STREAM_DEPTH:

                    status = _connected_device->GetIntegerNodeValue("STR_ExposureAutoROITop", value);
                    //printf("jesam %d Value roi min y %d\n", status, value);
                    status = status && _connected_device->GetIntegerNodeValue("STR_ExposureAutoROIBottom", value);
                    //printf("jesam %d Value roi max y %d\n", status, value);
                    status = status && _connected_device->GetIntegerNodeValue("STR_ExposureAutoROILeft", value);
                    //printf("jesam %d Value roi min x %d\n", status, value);
                    status = status && _connected_device->GetIntegerNodeValue("STR_ExposureAutoROIRight", value);
                    //printf("jesam %d Value roi max x %d\n", status, value);


                    _connected_device->SetStringNodeValue("STR_ExposureAuto", "Off");

                    value = roi.min_y;
                    status = _connected_device->SetIntegerNodeValue("STR_ExposureAutoROITop", value);

                    //printf("jesam %d Value roi min y %d\n", status, value);

                    value = roi.max_y;
                    status = status && _connected_device->SetIntegerNodeValue("STR_ExposureAutoROIBottom", value);

                    //printf("jesam %d Value roi max y %d\n", status, value);

                    value = roi.min_x;
                    status = status && _connected_device->SetIntegerNodeValue("STR_ExposureAutoROILeft", value);

                    //printf("jesam %d Value roi min x %d\n", status, value);

                    value = roi.max_x;
                    status = status && _connected_device->SetIntegerNodeValue("STR_ExposureAutoROIRight", value);

                    //printf("jesam %d Value roi max x %d\n", status, value);

                    _connected_device->SetStringNodeValue("STR_ExposureAuto", "On");

                    return status;
                case CS_STREAM_COLOR:
                    return false;
            }
        }

        control_range cs_device::get_pu_range(rs2_option option, cs_stream stream)
        {
            int32_t max, min, value;
            // Auto controls range is trimed to {0,1} range
            if(option == RS2_OPTION_ENABLE_AUTO_EXPOSURE || option == RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE ||
                    option == RS2_OPTION_BACKLIGHT_COMPENSATION || option == RS2_OPTION_EMITTER_ENABLED)
            {
                static const int32_t min = 0, max = 1, step = 1, def = 1;
                control_range range(min, max, step, def);

                return range;
            }

            if (!get_cs_param_value(option, value, stream)) value = 0;
            if (!get_cs_param_min(option, min, stream)) min = 0;
            if (!get_cs_param_max(option, max, stream)) max = 0;

            control_range range(min, max, get_cs_param_step(option, stream), value);

            return range;
        }

        bool cs_device::set_cs_param(rs2_option option, int32_t value, cs_stream stream)
        {
            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_HUE: return _connected_device->SetIntegerNodeValue(get_cs_param_name(option, stream),
                                                                                    (int)value);
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                case RS2_OPTION_EMITTER_ENABLED:
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                {
                    if (value == 1) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "On");
                    else if (value == 0) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "Off");
                }
                /*case RS2_OPTION_EXPOSURE: return _connected_device->SetFloatNodeValue(get_cs_param_name(option, stream),
                                                                                      (double)value);
                case RS2_OPTION_GAMMA: return _connected_device->SetFloatNodeValue(get_cs_param_name(option),
                                                                                   (double)value);
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                {
                    if (value == 1) return _connected_device->SetStringNodeValue(get_cs_param_name(option), "Continuous");
                    else if (value == 0) return _connected_device->SetStringNodeValue(get_cs_param_name(option), "Off");
                }*/
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        std::string cs_device::get_cs_param_name(rs2_option option, cs_stream stream)
        {
            switch(option)
            {
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_WhiteBalanceTempAuto");
                case RS2_OPTION_WHITE_BALANCE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_WhiteBalanceTemp");
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_ExposureAuto");
                    else if (stream == CS_STREAM_DEPTH) return std::string("STR_ExposureAuto");
                case RS2_OPTION_EXPOSURE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Exposure");
                    else if (stream == CS_STREAM_DEPTH) return std::string("STR_Exposure");
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_BacklightComp");
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_PowerLineFrequency");
                case RS2_OPTION_GAMMA:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Gamma");
                case RS2_OPTION_SHARPNESS:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Sharpness");
                case RS2_OPTION_SATURATION:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Saturation");
                case RS2_OPTION_BRIGHTNESS:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Brightness");
                case RS2_OPTION_CONTRAST:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Contrast");
                case RS2_OPTION_GAIN:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Gain");
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_Gain");
                case RS2_OPTION_HUE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Hue");
                case RS2_OPTION_EMITTER_ENABLED:
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_LaserEnable");
                case RS2_OPTION_LASER_POWER:
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_LaserPower");
                //case RS2_OPTION_EXPOSURE: return std::string("ExposureTime");
                //case RS2_OPTION_GAMMA: return std::string("Gamma");
                //case RS2_OPTION_ENABLE_AUTO_EXPOSURE: return std::string("ExposureAuto");
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_min(rs2_option option, int32_t &value, cs_stream stream)
        {
            double double_value;
            smcs::StringList node_value_list;
            INT64 int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    //if (_connected_device->IsImplemented(get_cs_param_name(option, stream))
                    status = _connected_device->GetIntegerNodeMin(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = 0;
                    return status;
                /*case RS2_OPTION_EXPOSURE:
                {
                    status = _connected_device->GetFloatNodeMin(get_cs_param_name(option, stream), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }
                case RS2_OPTION_GAMMA:
                {
                    status = _connected_device->GetFloatNodeMin(get_cs_param_name(option), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }*/
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_max(rs2_option option, int32_t &value, cs_stream stream)
        {
            double double_value;
            smcs::StringList node_value_list;
            INT64 int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    //if (_connected_device->IsImplemented(get_cs_param_name(option, stream))
                    status = _connected_device->GetIntegerNodeMax(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = static_cast<int32_t>(node_value_list.size()-1);
                    return status;
                /*case RS2_OPTION_EXPOSURE:
                {
                    status = _connected_device->GetFloatNodeMax(get_cs_param_name(option, stream), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }
                case RS2_OPTION_GAMMA:
                {
                    status = _connected_device->GetFloatNodeMax(get_cs_param_name(option), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }*/
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        int32_t cs_device::get_cs_param_step(rs2_option option, cs_stream stream)
        {
            INT64 int_value;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    _connected_device->GetIntegerNodeIncrement(get_cs_param_name(option, stream), int_value);
                    return static_cast<int32_t>(int_value);
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    return 1;
                //case RS2_OPTION_EXPOSURE: return 1;
                //case RS2_OPTION_GAMMA: return 1;
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_value(rs2_option option, int32_t &value, cs_stream stream)
        {
            double double_value;
            std::string string_value;
            INT64 int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    status = _connected_device->GetIntegerNodeValue(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                case RS2_OPTION_EMITTER_ENABLED:
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                {
                    status = _connected_device->GetStringNodeValue(get_cs_param_name(option, stream), string_value);
                    if (string_value == "Off") value = 0;
                    else value = 1;
                    return status;
                }
                /*case RS2_OPTION_EXPOSURE:
                {
                    status = _connected_device->GetFloatNodeValue(get_cs_param_name(option, stream), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }
                case RS2_OPTION_GAMMA:
                {
                    status = _connected_device->GetFloatNodeValue(get_cs_param_name(option), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                {
                    status = _connected_device->GetStringNodeValue(get_cs_param_name(option), string_value);
                    if (string_value.compare(std::string("Off")) == 0) value = 0;
                    else if (string_value.compare(std::string("Continuous")) == 0) value = 1;
                    return true;
                }*/
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        void cs_device::close(stream_profile profile, cs_stream_id stream)
        {
            if (stream < _number_of_streams)
            {
                if (_is_capturing[stream])
                {
                    _is_capturing[stream] = false;
                    stop_acquisition(stream);
                    _threads[stream]->join();
                    _threads[stream].reset();
                }
                if (_callbacks[stream]) _callbacks[stream] = nullptr;
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::start_acquisition(cs_stream_id stream)
        {
            // NOTE: selectors and controls for starting and stopping acquisition will change in future fw versions

            // select the appropriate source (TriggerMode, AcquisitionMode, TLParamsLocked, AcquisitionStart)
            _connected_device->SetIntegerNodeValue("SourceControlSelector", stream);

            // select the appropriate channel (GevSCPSPacketSize, GevSCPD)
            INT64 streamChannel;
            if (_connected_device->GetIntegerNodeValue("SourceStreamChannel", streamChannel)) {
                _connected_device->SetIntegerNodeValue("GevStreamChannelSelector", streamChannel);
            }

            // disable trigger mode
            _connected_device->SetStringNodeValue("TriggerMode", "Off");
            // set continuous acquisition mode
            _connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");
                
            // optimal settings for D435e
            _connected_device->SetIntegerNodeValue("GevSCPSPacketSize", 1500);
            _connected_device->SetIntegerNodeValue("GevSCPD", 10);

            // start acquisition
            _connected_device->SetIntegerNodeValue("TLParamsLocked", 1);
            _connected_device->CommandNodeExecute("AcquisitionStart");
        }

        void cs_device::stop_acquisition(cs_stream_id stream)
        {
            // select the appropriate source (TLParamsLocked, AcquisitionStart)
            _connected_device->SetIntegerNodeValue("SourceControlSelector", stream);

            _connected_device->CommandNodeExecute("AcquisitionStop");
            _connected_device->SetIntegerNodeValue("TLParamsLocked", 0);
        }

        void cs_device::stream_on(std::function<void(const notification& n)> error_handler, cs_stream_id stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream])
                {
                    _error_handler[stream] = error_handler;
                    _is_capturing[stream] = true;
                    start_acquisition(stream);
                    _threads[stream] = std::unique_ptr<std::thread>(new std::thread([this, stream](){ capture_loop(stream); }));
                }
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        power_state cs_device::set_power_state(power_state state)
        {
            std::lock_guard<std::mutex> lock(_power_lock);

            std::string serial;

            if (state == D0 && _power_state == D3)
            {
                _power_state = D0;
            }
            if (state == D3 && _power_state == D0)
            {
                _power_state = D3;
            }

            return _power_state;
        }

        bool cs_device::reset(void)
        {
            return _connected_device->CommandNodeExecute("DeviceReset");
        }

        std::vector<byte> cs_device::send_hwm(std::vector<byte>& buffer)
        {
            std::lock_guard<std::mutex> lock(_hwm_lock);
            
            UINT64 address, regLength;
            UINT32 restSize;
            UINT8 readBuffer[GVCP_WRITEMEM_MAX_COUNT];
            UINT8 resendCount = 3;
            double maxWaitTime = 5.2;
            UINT16 size = 0;
            GEV_STATUS gevStatus;
            bool status;

            std::vector<byte> out_vec;

            _connected_device->GetNode("STR_HWmTxBuffer")->GetRegisterNodeLength(regLength);
            _connected_device->GetNode("STR_HWmTxBuffer")->GetRegisterNodeAddress(address);
            _connected_device->SetMemory(address, buffer.size(), buffer.data(), &gevStatus, maxWaitTime);
            _connected_device->CommandNodeExecute("STR_HWmTxBufferSend");

            _connected_device->GetNode("STR_HWmRxBuffer")->GetRegisterNodeLength(regLength);
            restSize = regLength - 4;
            _connected_device->GetNode("STR_HWmRxBuffer")->GetRegisterNodeAddress(address);

            _connected_device->CommandNodeExecute("STR_HWmRxBufferReceive");
            // Receive HWm command
            while (restSize > 0) {

                size = (restSize > GVCP_WRITEMEM_MAX_COUNT) ? GVCP_WRITEMEM_MAX_COUNT : restSize;

                int tryCount = resendCount;
                bool ok;

                do {
                    ok = true;
                    tryCount--;
                    ok = ok && _connected_device->GetMemory(address, size, (UINT8*)(&readBuffer[0]), &gevStatus, maxWaitTime);
                    if (ok) {
                        // TODO use insert or std::copy instead
                        for (UINT16 i = 0; i < size; i++) {
                            out_vec.push_back(readBuffer[i]);
                        }
                    }
                } while ((tryCount > 0) && (!ok));

                if (!ok) {
                    status = false;
                    break;
                }

                restSize -= size;
                address += size;
            }
            return out_vec;
        }

        void cs_device::capture_loop(cs_stream_id stream)
        {
            try
            {
                while(_is_capturing[stream])
                {
                    image_poll(stream);
                }
            }
            catch (const std::exception& ex)
            {
                LOG_ERROR(ex.what());

                librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, ex.what()};

                _error_handler[stream](n);
            }
        }

        void cs_device::image_poll(cs_stream_id stream)
        {
            std::lock_guard<std::mutex> lock(_stream_lock);

            smcs::IImageInfo image_info_;

            UINT32 src_pixel_type;
            double timestamp;
            if (_connected_device.IsValid() && _connected_device->IsConnected() && _connected_device->IsOnNetwork()) {
                //if (!_connected_device->IsBufferEmpty()) {
                if (_connected_device->GetImageInfo(&image_info_, stream))
                {
                    auto image_id = image_info_->GetImageID();

                    if (cs_info::is_timestamp_supported(_device_info.id))
                        timestamp = image_info_->GetCameraTimestamp() / 1000000.0;
                    timestamp = -1;

                    auto im = image_info_->GetRawData();
                    image_info_->GetPixelType(src_pixel_type);
                    auto image_size = image_info_->GetRawDataSize();

                    frame_object fo {image_size, 0, im, NULL, timestamp};

                    _callbacks[stream](_profiles[stream], fo, []() {});

                    _connected_device->PopImage(image_info_);
                    //_connected_device->ClearImageBuffer();
                }
                //}
            }
        }

        void cs_device::probe_and_commit(stream_profile profile, frame_callback callback, cs_stream_id stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream]/* && !_callbacks[stream]*/)
                {
                    set_format(profile);
                    _profiles[stream] = profile;
                    _callbacks[stream] = callback;
                }
                else throw wrong_api_call_sequence_exception("Device already streaming!");
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::set_format(stream_profile profile)
        {
            //TODO
            //tu se odabire profil na kameri
        }
    }
}
