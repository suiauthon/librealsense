//
// Created by marko on 21.05.19..
//

#include "cs/cs-factory.h"
#include "cs/cs-device.h"

namespace librealsense {
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
        else if (pid.compare("UCC1932C") == 0) return CS_UCC1932C;
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
        printf("Trazim\n");
        smcs_api->FindAllDevices(0.1);
        printf("Nasao\n");
        auto devices = smcs_api->GetAllDevices();

        for (int i = 0; i < devices.size(); i++) {
            printf("Broj uredaja na kompu %d\n", devices.size());
            if (devices[i]->IsOnNetwork()) {
                printf("Uredaj je na kompu\n");
                auto info = platform::cs_device_info();
                info.serial = devices[i]->GetSerialNumber();
                info.id = devices[i]->GetModelName();
                printf("id %s\n", info.id.c_str());
                info.info = devices[i]->GetManufacturerSpecificInfo();

                results.push_back(info);
            }
        }

        return results;
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
                profile.format = 'YUYV';
            }
            if (_connected_device->GetIntegerNodeValue("FPS", int64Value)) {
                profile.fps = (uint32_t)int64Value;
            }
            else profile.fps = 50;
            all_stream_profiles.push_back(profile);
            profile.format = 'Z16 ';
            all_stream_profiles.push_back(profile);

            profile.format = 'GREY';
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
            if (stream < _number_of_streams)
            {
                if (_is_capturing[stream])
                {
                    _is_capturing[stream] = false;
                    stop__acquisition();
                    _threads[stream]->join();
                    _threads[stream].reset();
                }
                if (_callbacks[stream]) _callbacks[stream] = nullptr;
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::start_acquisition()
        {
            if (_is_acquisition_active == 0)
            {
                // disable trigger mode
                _connected_device->SetStringNodeValue("TriggerMode", "Off");
                // set continuous acquisition mode
                _connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");
                // start acquisition
                _connected_device->SetIntegerNodeValue("TLParamsLocked", 1);
                _connected_device->CommandNodeExecute("AcquisitionStart");
            }

            _is_acquisition_active++;

        }

        void cs_device::stop__acquisition()
        {
            if (_is_acquisition_active == 1)
            {
                _connected_device->CommandNodeExecute("AcquisitionStop");
                _connected_device->SetIntegerNodeValue("TLParamsLocked", 0);
            }

            _is_acquisition_active--;
            if (_is_acquisition_active < 0) _is_acquisition_active = 0;
        }

        void cs_device::stream_on(std::function<void(const notification& n)> error_handler, cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream])
                {
                    _error_handler[stream] = error_handler;
                    _is_capturing[stream] = true;
                    start_acquisition();
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

        void cs_device::capture_loop(cs_stream stream)
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

        void cs_device::image_poll(cs_stream stream)
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

        void cs_device::probe_and_commit(stream_profile profile, frame_callback callback, cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream] && !_callbacks[stream])
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
