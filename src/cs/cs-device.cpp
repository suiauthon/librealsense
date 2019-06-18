//
// Created by marko on 09.03.19..
//

#include "cs-device.h"
#include "cs-timestamp.h"

namespace librealsense
{
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
        printf("Cs sensor CLOSE\n");
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
              cs_camera(ctx, hwm_device, group, register_device_notifications),
              cs_mono(ctx, group, register_device_notifications)
    {
        _cs_device = ctx->get_backend().create_cs_device(hwm_device);
        _mono_device_idx = add_sensor(create_mono_device(ctx, _cs_device));
    }

    D435e_camera::D435e_camera(std::shared_ptr<context> ctx,
                               const platform::cs_device_info &hwm_device,
                               const platform::backend_device_group& group,
                               bool register_device_notifications)
            : device(ctx, group, register_device_notifications),
              cs_camera(ctx, hwm_device, group, register_device_notifications),
              cs_color(ctx, group, register_device_notifications),
              cs_depth(ctx, group, register_device_notifications)
    {
        _cs_device = ctx->get_backend().create_cs_device(hwm_device);

        _color_device_idx = add_sensor(create_color_device(ctx, _cs_device));
        _depth_device_idx = add_sensor(create_depth_device(ctx, _cs_device));
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
        std::vector<stream_interface*> streams = { _depth_stream.get(), _color_stream.get() };
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
}