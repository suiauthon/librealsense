//
// Created by marko on 21.05.19..
//

#include "cs/cs-sensor.h"
#include "global_timestamp_reader.h"
#include "stream.h"
#include "device.h"
#include <regex>

namespace librealsense {

    cs_firmware_version::cs_firmware_version(smcs::IDevice &device)
    {
        auto device_version = device->GetDeviceVersion();
        std::smatch match;
        if (std::regex_search(device_version, match, std::regex("FW:([0-9])\\.([0-9])\\.([0-9])\\.([0-9])")))
        {
            try
            {
                _major = std::stoi(match[1]);
                _minor = std::stoi(match[2]);
                _patch = std::stoi(match[3]);
                _build = std::stoi(match[4]);
            }
            catch (...)
            {
                _major = 0;
                _minor = 0;
                _patch = 0;
                _build = 0;
            }
        }

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

        _cs_selected_streams.clear();

        for (auto&& mode : mapping)
        {
            for (auto&& request : mode.original_requests)
            {
                try
                {
                    auto selected_stream = get_stream(request->get_stream_type(), request->get_stream_index());

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
                                << " ,diff_ts[Sys-BE]," << system_time - f.backend_time
                                << ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain)
                                << ",last_frame_number," << last_frame_number << ",last_timestamp," << last_timestamp);

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
                            unpacker.unpack(dest.data(), reinterpret_cast<const byte *>(f.pixels), mode.profile.width, mode.profile.height, f.frame_size);
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

                    }, selected_stream);
                    _cs_selected_streams.push_back(selected_stream);
                }
                catch (...)
                {
                    for (auto&& commited_profile : commited)
                    {
                        for (auto stream : _cs_selected_streams)
                            _device->close(commited_profile, stream);
                    }
                    throw;
                }
                commited.push_back(mode.profile);
            }
        }

        _internal_config = commited;

        if (_on_open)
            _on_open(_internal_config);

        _power = move(on);
        _is_opened = true;

        try {

            for (auto stream : _cs_selected_streams)
                _device->stream_on([&](const notification& n)
                               {
                                   _notifications_processor->raise_notification(n);
                               }, stream);
        }
        catch (...)
        {
            for (auto& profile : _internal_config)
            {
                try {
                    for (auto stream : _cs_selected_streams)
                        _device->close(profile, stream);
                }
                catch (...) {}
            }
            reset_streaming();
            _power.reset();
            _is_opened = false;
            throw;
        }
        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(true);
        }
        set_active_streams(requests);
    }

    stream_profiles cs_sensor::init_stream_profiles()
    {
        OutputDebugStringA("init_stream_profiles\n");

        auto lock = environment::get_instance().get_extrinsics_graph().lock();

        std::unordered_set<std::shared_ptr<video_stream_profile>> results;
        std::set<uint32_t> unregistered_formats;
        std::set<uint32_t> supported_formats;

        power on(std::dynamic_pointer_cast<cs_sensor>(shared_from_this()));

        if (_uvc_profiles.empty()){}
        _uvc_profiles = _device->get_profiles(_cs_stream);

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
            for (auto& elem : supported_formats)
            {
                uint32_t device_fourcc = reinterpret_cast<const big_endian<uint32_t>&>(elem);
                char fourcc[sizeof(device_fourcc) + 1];
                librealsense::copy(fourcc, &device_fourcc, sizeof(device_fourcc));
                fourcc[sizeof(device_fourcc)] = 0;
                ss << fourcc << " ";
            }
            ss << "]";
            LOG_INFO(ss.str());
        }

        auto rgbFormat = _device->get_rgb_format();

        // Sort the results to make sure that the user will receive predictable deterministic output from the API
        stream_profiles res{ begin(results), end(results) };
        std::sort(res.begin(), res.end(), [&rgbFormat](const std::shared_ptr<stream_profile_interface>& ap,
                                             const std::shared_ptr<stream_profile_interface>& bp)
        {
            auto a = to_profile(ap.get());
            auto b = to_profile(bp.get());

            // stream == RS2_STREAM_COLOR && format == rgbFormat element works around the fact that Y16 gets priority over RGB8 when both
            // are available for pipeline stream resolution
            auto at = std::make_tuple(a.stream, a.width, a.height, a.fps, a.stream == RS2_STREAM_COLOR && a.format == rgbFormat, a.format);
            auto bt = std::make_tuple(b.stream, b.width, b.height, b.fps, b.stream == RS2_STREAM_COLOR && b.format == rgbFormat, b.format);

            return at > bt;
        });

        return res;
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
            try // Handle disconnect event
            {
                for (auto stream : _cs_selected_streams)
                    _device->close(profile, stream);
            }
            catch (...) {}
        }
        reset_streaming();
        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(false);
        }
        _power.reset();
        _is_opened = false;
        set_active_streams({});
    }

    void cs_sensor::start(frame_callback_ptr callback)
    {
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

    cs_stream cs_sensor::get_stream(rs2_stream type, int index)
    {
        switch (type)
        {
        case RS2_STREAM_DEPTH:
            return CS_STREAM_DEPTH;
        case RS2_STREAM_COLOR:
            return CS_STREAM_COLOR;
        case RS2_STREAM_INFRARED:
            return index == 1 ? CS_STREAM_IR_LEFT : CS_STREAM_IR_RIGHT;
        default:
            throw wrong_api_call_sequence_exception("unable to map type and index to stream ID!");
        }
    }

    void cs_sensor::register_pu(rs2_option id)
    {
        register_option(id, std::make_shared<cs_pu_option>(*this, id, _cs_stream));
    }

    void cs_sensor::try_register_pu(rs2_option id)
    {
        auto opt = std::make_shared<cs_pu_option>(*this, id, _cs_stream);
        try
        {
            auto range = opt->get_range();
            if (range.max <= range.min || range.step <= 0 || range.def < range.min || range.def > range.max) return;

            auto val = opt->query();
            if (val < range.min || val > range.max) return;
            opt->set(val);

            register_option(id, opt);
        }
        catch (...)
        {
            LOG_WARNING("Exception was thrown when inspecting " << this->get_info(RS2_CAMERA_INFO_NAME) << " property " << opt->get_description());
        }
    }

    namespace platform
    {
        enum rs2_format cs_device::get_rgb_format()
        {
            if (_cs_firmware_version >= cs_firmware_version(1, 3, 4, 2))
                return RS2_FORMAT_RGB8;
            else
                return RS2_FORMAT_BGR8;
        }

        std::vector<stream_profile> cs_device::get_profiles(cs_stream stream)
        {
            std::vector<stream_profile> all_stream_profiles;
            stream_profile profile;
            INT64 int64_value;
            bool is_successful = true;

            //TODO preserving this values is probably not requireds
            /*std::string sourceSelectorValue;
            is_successful = is_successful &_connected_device->GetStringNodeValue("SourceControlSelector", sourceSelectorValue);

            std::string resolutionValue;
            if (_cs_firmware_version >= cs_firmware_version(1, 3))
                is_successful = is_successful &_connected_device->GetStringNodeValue("Resolution", resolutionValue);
            else
                resolutionValue = "Res_1280x720";*/

            //TODO disable all regions

            OutputDebugStringA("get profiles\n");
            disable_source_regions(stream);
            auto test = set_region(stream, true);
            if (!test) {
                OutputDebugStringA("select_region failed in enum\n");
            }

            smcs::StringList resolution_list;
            if (_cs_firmware_version >= cs_firmware_version(1, 3, 4, 2))
                is_successful = is_successful & _connected_device->GetEnumNodeValuesList("Resolution", resolution_list);
            else
                resolution_list.push_back("Res_1280x720");

            if (!is_successful)
                OutputDebugStringA("failed to get resolution node\n");

            for (const auto& resolution : resolution_list) {

                if (_cs_firmware_version >= cs_firmware_version(1, 3, 4, 2))
                    is_successful =
                        is_successful & _connected_device->SetStringNodeValue("Resolution", resolution);

                if (!is_successful)
                    OutputDebugStringA("failed to set node Resolution\n");
                else
                    OutputDebugStringA("setting resolution node ok\n");

                is_successful = is_successful & _connected_device->GetIntegerNodeValue("Width", int64_value);
                profile.width = (uint32_t) int64_value;

                if (!is_successful)
                    OutputDebugStringA("failed to set node width \n");

                is_successful = is_successful & _connected_device->GetIntegerNodeValue("Height", int64_value);
                profile.height = (uint32_t) int64_value;

                std::string pixelFormat;
                is_successful = is_successful & _connected_device->GetStringNodeValue("PixelFormat", pixelFormat);

                if (!is_successful)
                    OutputDebugStringA("failed to set res...\n");

                profile.format = cs_pixel_format_to_native_pixel_format(pixelFormat);

                auto frameRates = get_frame_rates();

                for (auto frameRate : frameRates) {

                    profile.fps = frameRate;

                    if (is_successful) {
                        profile.format = cs_pixel_format_to_native_pixel_format(pixelFormat);
                        all_stream_profiles.push_back(profile);
                            
                        std::string source;
                        is_successful = is_successful &_connected_device->GetStringNodeValue("SourceControlSelector", source);
            
                        if (is_successful && source == "Source0") {
                            profile.format = 'Y8I ';
                            all_stream_profiles.push_back(profile);
                        }
                    }
                }
            }

            //TODO preserving setting not required
            /*_connected_device->SetStringNodeValue("SourceControlSelector", sourceSelectorValue);
            if (_cs_firmware_version >= cs_firmware_version(1, 3, 4, 2))
                _connected_device->SetStringNodeValue("Resolution", resolutionValue);*/
            
            set_region(stream, false);

            return all_stream_profiles;
        }

        std::vector<uint32_t> cs_device::get_frame_rates()
        {
            if (_cs_firmware_version >= cs_firmware_version(1, 3, 4, 2))
                return get_frame_rates_from_control();
            else
                return std::vector<uint32_t> {30};
        }

        std::vector<uint32_t> cs_device::get_frame_rates_from_control()
        {
            std::string frameRateValue;
            _connected_device->GetStringNodeValue("FrameRate", frameRateValue);

            smcs::StringList frameRates;
            _connected_device->GetEnumNodeValuesList("FrameRate", frameRates);

            std::vector<uint32_t> acquisitionFrameRates;
            for (const auto& frameRate : frameRates) {
                _connected_device->SetStringNodeValue("FrameRate", frameRate);

                double acquisitionFrameRate;
                if (_connected_device->GetFloatNodeValue("AcquisitionFrameRate", acquisitionFrameRate))
                    acquisitionFrameRates.push_back(static_cast<uint32_t>(acquisitionFrameRate));
            }

            _connected_device->SetStringNodeValue("FrameRate", frameRateValue);

            return acquisitionFrameRates;
        }

        uint32_t cs_device::cs_pixel_format_to_native_pixel_format(std::string cs_format)
        {
            uint32_t npf;
            if (cs_format.compare("YUV422") == 0)
                npf = rs_fourcc('Y', 'U', 'Y', 'V');
            else if (cs_format.compare("YUV422Packed") == 0)
                npf = rs_fourcc('U', 'Y', 'V', 'Y');
            else if (cs_format.compare("Mono16") == 0)
                npf = rs_fourcc('Z','1','6',' ');
            else 
                throw wrong_api_call_sequence_exception("Unsuported image format!");

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

            //range copied from asic_and_projector_temperature_options::get_range()
            if (option == RS2_OPTION_ASIC_TEMPERATURE || option == RS2_OPTION_PROJECTOR_TEMPERATURE)
                return control_range{ -40, 125, 0, 0 };

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
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    return _connected_device->SetIntegerNodeValue(get_cs_param_name(option, stream),
                        (int)value);
                }
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
                case RS2_OPTION_INTER_PACKET_DELAY: return std::string("GevSCPD");
                case RS2_OPTION_PACKET_SIZE: return std::string("GevSCPSPacketSize");
                case RS2_OPTION_ASIC_TEMPERATURE: return std::string("IntelASIC");
                case RS2_OPTION_PROJECTOR_TEMPERATURE: return std::string("DepthModule");
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
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeMin(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
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
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeMax(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
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
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return 1;

                    _connected_device->GetIntegerNodeIncrement(get_cs_param_name(option, stream), int_value);
                    return static_cast<int32_t>(int_value);
                }
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
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeValue(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_ASIC_TEMPERATURE:
                case RS2_OPTION_PROJECTOR_TEMPERATURE:
                {
                    if (_connected_device->SetStringNodeValue("DeviceTemperatureSelector", get_cs_param_name(option, stream))) {
                        double temperature;
                        if (_connected_device->GetFloatNodeValue("DeviceTemperature", temperature)) {
                            value = static_cast<int32_t> (temperature);
                            return true;
                        }
                    }
                    return false;
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        void cs_device::close(stream_profile profile, cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                stop_stream(stream);
                if (_callbacks[stream]) _callbacks[stream] = nullptr;
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::start_acquisition(cs_stream stream)
        {
            std::stringstream ss;
            ss << "start acquisition " << stream << "\n";
            OutputDebugStringA(ss.str().c_str());

            if (_cs_firmware_version <= cs_firmware_version(1, 2, 0, 0)) {

                auto capturing = std::count_if(
                    _is_capturing.begin(), _is_capturing.end(), 
                    [](std::atomic<bool> &capturing) {
                        return capturing == true;
                    }
                );
                if (capturing != 1)
                    return;
            }            
            
            auto test = set_region(stream, true);
            if (!test) {
                OutputDebugStringA("failed to set region!!!!\n");
            }

            // disable trigger mode
            _connected_device->SetStringNodeValue("TriggerMode", "Off");
            // set continuous acquisition mode
            _connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");

            // start acquisition
            _connected_device->SetIntegerNodeValue("TLParamsLocked", 1);
            auto t2 = _connected_device->CommandNodeExecute("AcquisitionStart");
            if (!t2) {
                OutputDebugStringA("fail to start acq\n");
            }
        }

        void cs_device::stop_stream(cs_stream stream)
        {
            if (_is_capturing[stream])
            {
                _is_capturing[stream] = false;
                stop_acquisition(stream);
                _threads[stream]->join();
                _threads[stream].reset();
            }
        }

        void cs_device::stop_acquisition(cs_stream stream)
        {
            if (_cs_firmware_version <= cs_firmware_version(1, 2, 0, 0)) {

                auto capturing = std::find_if(
                    _is_capturing.begin(), _is_capturing.end(), 
                    [](std::atomic<bool> &capturing) { 
                        return capturing == true; 
                    }
                );
                if (capturing != _is_capturing.end())
                    return;
            }

            select_source(stream);

            _connected_device->CommandNodeExecute("AcquisitionStop");
            _connected_device->SetIntegerNodeValue("TLParamsLocked", 0);

            set_region(stream, false);
        }

        bool cs_device::select_source(cs_stream stream)
        {
            return _connected_device->SetIntegerNodeValue("SourceControlSelector", get_stream_source(stream));
        }

        bool cs_device::set_region(cs_stream stream, bool enable)
        {
            if (select_region(stream))
                return _connected_device->SetStringNodeValue("RegionMode", enable ? "On" : "Off");

            OutputDebugStringA("setting region failed\n");
            return false;
        }

        bool cs_device::disable_source_regions(cs_stream stream)
        {
            if (!select_source(stream))
                return false;

            smcs::StringList regions;
            if (!_connected_device->GetEnumNodeValuesList("RegionSelector", regions))
                return false;

            for (const auto& region : regions)
                _connected_device->SetStringNodeValue("RegionSelector", region);
                //set off

            return true;
        }

        bool cs_device::select_region(cs_stream stream)
        {
            if (select_source(stream))
                return _connected_device->SetIntegerNodeValue("RegionSelector", get_stream_region(stream));

            OutputDebugStringA("failed to select region\n");
            return false;
        }

        bool cs_device::select_channel(cs_stream stream)
        {
            //TOOD preserved to help with compatibility with older fw
            /*if (!_connected_device->SetIntegerNodeValue("SourceControlSelector", get_stream_source(stream)))
            return false;

            INT64 stream_channel;
            if (!_connected_device->GetIntegerNodeValue("SourceStreamChannel", stream_channel))
            return false;

            if (!_connected_device->SetIntegerNodeValue("GevStreamChannelSelector", stream_channel))
            return false;*/

            UINT32 channel;
            if (get_stream_channel(stream, channel))
                return _connected_device->SetIntegerNodeValue("GevStreamChannelSelector", static_cast<INT64>(channel));

            return false;
        }

        INT64 cs_device::get_stream_source(cs_stream stream)
        {
            switch (stream) {
            case CS_STREAM_DEPTH:
            case CS_STREAM_IR_LEFT:
            case CS_STREAM_IR_RIGHT:
                return 0;
            case CS_STREAM_COLOR:
                return 1;
            default:
                throw wrong_api_call_sequence_exception("Unknown stream ID!");
            }
        }

        INT64 cs_device::get_stream_region(cs_stream stream)
        {
            switch (stream) {
            case CS_STREAM_DEPTH:
            case CS_STREAM_COLOR:
                return 0;
            case CS_STREAM_IR_LEFT:
                return 3;
            case CS_STREAM_IR_RIGHT:
                return 2;
            default:
                throw wrong_api_call_sequence_exception("Unknown stream ID!");
            }
        }

        //TODO will these results change dynamically? consider caching them in some way...
        bool cs_device::get_stream_channel(cs_stream stream, UINT32& channel)
        {
            if (!select_source(stream))
                return false;

            if (!select_region(stream))
                return false;

            std::string region_destination;
            if (!_connected_device->GetStringNodeValue("RegionDestination", region_destination))
                return false;

            if (!_connected_device->SetStringNodeValue("TransferSelector", region_destination))
                return false;

            INT64 transfer_stream_channel;
            if (!_connected_device->GetIntegerNodeValue("TransferStreamChannel", transfer_stream_channel))
                return false;

            channel = static_cast<UINT32>(transfer_stream_channel);
            return true;
        }

        void cs_device::stream_on(std::function<void(const notification& n)> error_handler, cs_stream stream)
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

            //SETRGBAEROI, see enum fw_cmd ind5-private.h
            const uint8_t setrgbaeroi_opcode = 0x75;
            const uint8_t unknown_opcode = 0x00;
            const uint8_t opcode_index = 4;
            const uint8_t opcode = buffer.size() > opcode_index ? buffer[opcode_index] : unknown_opcode;
            if ((opcode == setrgbaeroi_opcode) && (buffer.size() >= (6 * sizeof(uint32_t)))) {
                uint32_t param_index = 8;
                uint32_t param1 = read_from_buffer(buffer, param_index);
                uint32_t param2 = read_from_buffer(buffer, param_index + 4);
                uint32_t param3 = read_from_buffer(buffer, param_index + 8);
                uint32_t param4 = read_from_buffer(buffer, param_index + 12);
                set_rgb_ae_roi(param1, param2, param3, param4);
                return std::vector<byte> {setrgbaeroi_opcode, 0, 0, 0};
            }
            else {
                return send_hwm_to_device(buffer);
            }
        }

        uint32_t cs_device::read_from_buffer(std::vector<byte>& buffer, uint32_t index)
        {
            uint32_t result = 0;

            for (int i = 0; i < sizeof(uint32_t); ++i) {
                result += static_cast<uint32_t>(buffer[index + i]) << (i * 8);
            }

            return result;
        }

        std::vector<byte> cs_device::send_hwm_to_device(std::vector<byte>& buffer)
        {
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

        void cs_device::set_rgb_ae_roi(uint32_t top, uint32_t bottom, uint32_t left, uint32_t right)
        {
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROITop", top);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROILeft", left);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROIBottom", bottom);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROIRight", right);
            _connected_device->CommandNodeExecute("RGB_ExposureAutoROISet");
        }

        std::string cs_device::get_device_version()
        {
            return _device_version;
        }

        bool cs_device::is_temperature_supported()
        {
            return _cs_firmware_version >= cs_firmware_version(1, 4, 1, 0);
        }

        void cs_device::capture_loop(cs_stream stream)
        {
            try
            {
                UINT32 channel;
                if (!get_stream_channel(stream, channel))
                    throw wrong_api_call_sequence_exception("Unable to get stream channel!");
                
                while(_is_capturing[stream])
                {
                    image_poll(stream, channel);
                }
            }
            catch (const std::exception& ex)
            {
                LOG_ERROR(ex.what());

                librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, ex.what()};

                _error_handler[stream](n);
            }
        }

        void cs_device::image_poll(cs_stream stream, UINT32 channel)
        {
            smcs::IImageInfo image_info_;

            UINT32 src_pixel_type;
            double timestamp;
            if (_connected_device.IsValid() && _connected_device->IsConnected() && _connected_device->IsOnNetwork()) {
                if (_connected_device->WaitForImage(3, channel))
                {
                    _connected_device->GetImageInfo(&image_info_, channel);
                    auto image_id = image_info_->GetImageID();

                    //if (cs_info::is_timestamp_supported(_device_info.id))
                    //    timestamp = image_info_->GetCameraTimestamp() / 1000000.0;
                    timestamp = -1;

                    auto im = image_info_->GetRawData();
                    image_info_->GetPixelType(src_pixel_type);
                    auto image_size = image_info_->GetRawDataSize();

                    frame_object fo {image_size, 0, im, NULL, timestamp};
                    
                    {
                        std::lock_guard<std::mutex> lock(_stream_lock);
                        _callbacks[stream](_profiles[stream], fo, []() {});
                    }

                    _connected_device->PopImage(image_info_);
                }
            }
        }

        void cs_device::probe_and_commit(stream_profile profile, frame_callback callback, cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream]/* && !_callbacks[stream]*/)
                {
                    set_format(profile, stream);
                    _profiles[stream] = profile;
                    _callbacks[stream] = callback;
                }
                else {
                    throw wrong_api_call_sequence_exception("Device already streaming!");
                }
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::set_format(stream_profile profile, cs_stream stream)
        {
            std::string sourceSelectorValue;
            _connected_device->GetStringNodeValue("SourceControlSelector", sourceSelectorValue);
            _connected_device->SetIntegerNodeValue("SourceControlSelector", stream);

            _connected_device->SetStringNodeValue("Resolution", "Res_" + std::to_string(profile.width) + "x" + std::to_string(profile.height));
            _connected_device->SetStringNodeValue("FrameRate", "FPS_" + std::to_string(profile.fps));

            _connected_device->SetStringNodeValue("SourceControlSelector", sourceSelectorValue);
        }
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
            case RS2_OPTION_INTER_PACKET_DELAY: return "Set inter-packet delay";
            case RS2_OPTION_PACKET_SIZE: return "Set packet size";
            case RS2_OPTION_ASIC_TEMPERATURE: return "Current Asic Temperature (degree celsius)";
            case RS2_OPTION_PROJECTOR_TEMPERATURE: return "Current Projector Temperature (degree celsius)";
            default: return _ep.get_option_name(_id);
        }
    }
}
