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
        printf("Trazim\n");
        smcs_api->FindAllDevices(0.5);
        printf("Nasao\n");
        auto devices = smcs_api->GetAllDevices();

        for (int i = 0; i < devices.size(); i++)
        {
            printf("Broj uredaja na kompu %d\n", devices.size());
            if (devices[i]->IsOnNetwork())
            {
                printf("Uredaj je na kompu\n");
                auto info = platform::cs_device_info();
                info.serial = devices[i]->GetSerialNumber();
                info.id = devices[i]->GetModelName();
                info.info = devices[i]->GetManufacturerSpecificInfo();

                results.push_back(info);
            }
        }

        return results;
    }

    cs_camera::cs_camera(std::shared_ptr<context> ctx,
                         const platform::cs_device_info &hwm_device,
                         const platform::backend_device_group& group,
                         bool register_device_notifications)
        : device(ctx, group, register_device_notifications),
          _color_stream(new stream(RS2_STREAM_COLOR)),
          _depth_stream(new stream(RS2_STREAM_DEPTH, 1))
    {
        _cs_device = ctx->get_backend().create_cs_device(hwm_device);

        _color_device_idx = add_sensor(create_color_device(ctx, _cs_device));
        _depth_device_idx = add_sensor(create_depth_device(ctx, _cs_device));

        register_info(RS2_CAMERA_INFO_NAME, hwm_device.info);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, hwm_device.id);
    }

    std::shared_ptr<matcher> cs_camera::create_matcher(const frame_holder& frame) const
    {
        std::vector<stream_interface*> streams = { _depth_stream.get(), _color_stream.get() };
        //return std::make_shared<identity_matcher>( frame.frame->get_stream()->get_unique_id(), frame.frame->get_stream()->get_stream_type());
        return matcher_factory::create(RS2_MATCHER_DEFAULT, streams);
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
        printf("Cs sensor OPEN\n");
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
                                                  /*if (!requires_processing)
                                                  {
                                                      pref->attach_continuation(std::move(release_and_enqueue));
                                                  }*/

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

                                          }, _cs_stream);

            }
            catch(...)
            {
                for (auto&& commited_profile : commited)
                {
                    _device->close(commited_profile, _cs_stream);
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
                               }, _cs_stream);
        }
        catch (...)
        {
            for (auto& profile : _internal_config)
            {
                try {
                    _device->close(profile, _cs_stream);
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
        printf("Cs sensor CLOSE\n");
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("close() failed. CS device is streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("close() failed. CS device was not opened!");

        for (auto& profile : _internal_config)
        {
            _device->close(profile, _cs_stream);
        }
        reset_streaming();
        _power.reset();
        _is_opened = false;
        set_active_streams({});
    }

    void cs_sensor::start(frame_callback_ptr callback)
    {
        printf("Cs sensor START\n");
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
        printf("Cs sensor STOP\n");
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
        register_option(id, std::make_shared<cs_pu_option>(*this, id));
    }

    void cs_sensor::try_register_pu(rs2_option id)
    {
        try
        {
            auto opt = std::make_shared<cs_pu_option>(*this, id);
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
            std::string node_value;

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

            return all_stream_profiles;
        }


        bool cs_device::get_pu(rs2_option opt, int32_t& value)
        {
            return get_cs_param_value(opt, value);
        }

        bool cs_device::set_pu(rs2_option opt, int32_t value)
        {
            return set_cs_param(opt, value);
        }

        control_range cs_device::get_pu_range(rs2_option option)
        {
            int32_t max, min, value;
            // Auto controls range is trimed to {0,1} range
            if(option >= RS2_OPTION_ENABLE_AUTO_EXPOSURE && option <= RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)
            {
                static const int32_t min = 0, max = 1, step = 1, def = 1;
                control_range range(min, max, step, def);

                return range;
            }

            if (!get_cs_param_value(option, value)) value = 0;
            if (!get_cs_param_min(option, min)) min = 0;
            if (!get_cs_param_max(option, max)) max = 0;

            control_range range(min, max, get_cs_param_step(option), value);

            return range;
        }

        bool cs_device::set_cs_param(rs2_option option, int32_t value)
        {
            switch(option)
            {
                case RS2_OPTION_EXPOSURE: return _connected_device->SetFloatNodeValue(get_cs_param_name(option),
                                                                                      (double)value);
                case RS2_OPTION_GAMMA: return _connected_device->SetFloatNodeValue(get_cs_param_name(option),
                                                                                      (double)value);
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                {
                    if (value == 1) return _connected_device->SetStringNodeValue(get_cs_param_name(option), "Continuous");
                    else if (value == 0) return _connected_device->SetStringNodeValue(get_cs_param_name(option), "Off");
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        std::string cs_device::get_cs_param_name(rs2_option option)
        {
            switch(option)
            {
                case RS2_OPTION_EXPOSURE: return std::string("ExposureTime");
                case RS2_OPTION_GAMMA: return std::string("Gamma");
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE: return std::string("ExposureAuto");
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_min(rs2_option option, int32_t &value)
        {
            double double_value;
            int int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_EXPOSURE:
                {
                    status = _connected_device->GetFloatNodeMin(get_cs_param_name(option), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }
                case RS2_OPTION_GAMMA:
                {
                    status = _connected_device->GetFloatNodeMin(get_cs_param_name(option), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_max(rs2_option option, int32_t &value)
        {
            double double_value;
            int int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_EXPOSURE:
                {
                    status = _connected_device->GetFloatNodeMax(get_cs_param_name(option), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }
                case RS2_OPTION_GAMMA:
                {
                    status = _connected_device->GetFloatNodeMax(get_cs_param_name(option), double_value);
                    value = static_cast<int32_t>(double_value);
                    return status;
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        int32_t cs_device::get_cs_param_step(rs2_option option)
        {
            switch(option)
            {
                case RS2_OPTION_EXPOSURE: return 1;
                case RS2_OPTION_GAMMA: return 1;
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_value(rs2_option option, int32_t &value)
        {
            double double_value;
            std::string string_value;
            int int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_EXPOSURE:
                {
                    status = _connected_device->GetFloatNodeValue(get_cs_param_name(option), double_value);
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
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        void cs_device::close(stream_profile profile, cs_stream stream)
        {
            switch(stream)
            {
                case CS_STREAM_COLOR:
                {
                    if (_is_color_capturing)
                    {
                        _is_color_capturing = false;

                        _connected_device->CommandNodeExecute("AcquisitionStop");
                        _connected_device->SetIntegerNodeValue("TLParamsLocked", 0);

                        _color_thread->join();
                        _color_thread.reset();
                    }
                    if (_color_callback) _color_callback = nullptr;
                    break;
                }
                case CS_STREAM_DEPTH:
                {
                    if (_is_depth_capturing)
                    {
                        _is_depth_capturing = false;

                        _connected_device->CommandNodeExecute("AcquisitionStop");
                        _connected_device->SetIntegerNodeValue("TLParamsLocked", 0);

                        _depth_thread->join();
                        _depth_thread.reset();
                    }
                    if (_depth_callback) _depth_callback = nullptr;
                    break;
                }
                default: throw wrong_api_call_sequence_exception("Unsuported streaming type!");
            }
        }

        void cs_device::stream_on(std::function<void(const notification& n)> error_handler, cs_stream stream)
        {
            switch(stream)
            {
                case CS_STREAM_COLOR:
                {
                    if (!_is_color_capturing)
                    {
                        _color_error_handler = error_handler;
                        _is_color_capturing = true;

                        // disable trigger mode
                        _connected_device->SetStringNodeValue("TriggerMode", "Off");
                        // set continuous acquisition mode
                        _connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");
                        // start acquisition
                        _connected_device->SetIntegerNodeValue("TLParamsLocked", 1);
                        _connected_device->CommandNodeExecute("AcquisitionStart");

                        _color_thread = std::unique_ptr<std::thread>(new std::thread([this](){ color_capture_loop(); }));
                    }
                    break;
                }
                case CS_STREAM_DEPTH:
                {
                    if (!_is_depth_capturing)
                    {
                        _depth_error_handler = error_handler;
                        _is_depth_capturing = true;

                        // disable trigger mode
                        _connected_device->SetStringNodeValue("TriggerMode", "Off");
                        // set continuous acquisition mode
                        _connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");
                        // start acquisition
                        _connected_device->SetIntegerNodeValue("TLParamsLocked", 1);
                        _connected_device->CommandNodeExecute("AcquisitionStart");

                        _depth_thread = std::unique_ptr<std::thread>(new std::thread([this](){ depth_capture_loop(); }));
                    }
                    break;
                }
                default: throw wrong_api_call_sequence_exception("Unsuported streaming type!");
            }
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

        void cs_device::color_capture_loop()
        {
            try
            {
                while(_is_color_capturing)
                {
                    color_image_poll();
                }
            }
            catch (const std::exception& ex)
            {
                LOG_ERROR(ex.what());

                librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, ex.what()};

                _color_error_handler(n);
            }
        }

        void cs_device::depth_capture_loop()
        {
            try
            {
                while(_is_depth_capturing)
                {
                    depth_image_poll();
                }
            }
            catch (const std::exception& ex)
            {
                LOG_ERROR(ex.what());

                librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, ex.what()};

                _color_error_handler(n);
            }
        }

        void cs_device::color_image_poll()
        {
            std::lock_guard<std::mutex> lock(_stream_lock);

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

                    _color_callback(_color_profile, fo, NULL);

                    _connected_device->PopImage(image_info_);
                    _connected_device->ClearImageBuffer();
                }
            }
        }

        void cs_device::depth_image_poll()
        {
            std::lock_guard<std::mutex> lock(_stream_lock);

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

                    _depth_callback(_depth_profile, fo, NULL);

                    _connected_device->PopImage(image_info_);
                    _connected_device->ClearImageBuffer();
                }
            }
        }

        void cs_device::probe_and_commit(stream_profile profile, frame_callback callback, cs_stream stream)
        {
            switch(stream)
            {
                case CS_STREAM_COLOR:
                {
                    if(!_is_color_capturing && !_color_callback)
                    {
                        set_format(profile);
                        _color_profile =  profile;
                        _color_callback = callback;
                    }
                    else throw wrong_api_call_sequence_exception("Device already streaming!");
                    break;
                }
                case CS_STREAM_DEPTH:
                {
                    if(!_is_depth_capturing && !_depth_callback)
                    {
                        set_format(profile);
                        _depth_profile =  profile;
                        _depth_callback = callback;
                    }
                    else throw wrong_api_call_sequence_exception("Device already streaming!");
                    break;
                }
                default: throw wrong_api_call_sequence_exception("Unsuported streaming type!");
            }
        }

        void cs_device::set_format(stream_profile profile)
        {
            //TODO
            //tu se odabire profil na kameri
        }
    }

    void cs_pu_option::set(float value)
    {
        _ep.invoke_powered(
            [this, value](platform::cs_device& dev)
            {
                if (!dev.set_pu(_id, static_cast<int32_t>(value)))
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
                if (!dev.get_pu(_id, value))
                    throw invalid_value_exception(to_string() << "get_pu(id=" << std::to_string(_id) << ") failed!" << " Last Error: " << strerror(errno));
                return static_cast<float>(value);
            }));
    }

    option_range cs_pu_option::get_range() const
    {
        auto cs_range = _ep.invoke_powered(
            [this](platform::cs_device& dev)
            {
                return dev.get_pu_range(_id);
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

}