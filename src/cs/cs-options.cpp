// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs-options.h"

namespace librealsense
{
    cs_asic_and_projector_temperature_options::cs_asic_and_projector_temperature_options(cs_sensor& ep, rs2_option opt, cs_stream stream)
            : _option(opt), _ep(ep), _stream(stream)
    {}

    const char* cs_asic_and_projector_temperature_options::get_description() const
    {
        switch (_option)
        {
            case RS2_OPTION_ASIC_TEMPERATURE:
                return "Current Asic Temperature (degree celsius)";
            case RS2_OPTION_PROJECTOR_TEMPERATURE:
                return "Current Projector Temperature (degree celsius)";
            default:
                throw invalid_value_exception(to_string() << _ep.get_option_name(_option) << " is not temperature option!");
        }
    }

    bool cs_asic_and_projector_temperature_options::is_enabled() const
    {
        //provjeriti da li treba dodati da li je omoguceno firmwareom ili nije
        return _ep.is_streaming();
    }

    option_range cs_asic_and_projector_temperature_options::get_range() const
    {
        //staviti da se cita iz kamere
        return option_range { -40, 125, 0, 0 };
    }

    float cs_asic_and_projector_temperature_options::query() const
    {
        if (!is_enabled())
            throw wrong_api_call_sequence_exception("query option is allow only in streaming!");

        #pragma pack(push, 1)
        struct temperature
        {
            uint8_t is_valid;
            int32_t temperature;
        };
        #pragma pack(pop)

        auto temperature_data = static_cast<temperature>( _ep.invoke_powered(
                [this](platform::cs_device& dev)
                {
                    temperature temp{};
                    if (!dev.get_pu(_option, temp.temperature, _stream)) temp.is_valid = 0;
                    else temp.is_valid = 1;
                    return temp;
                }));

        int32_t temperature::* field;
        uint8_t temperature::* is_valid_field;

        switch (_option)
        {
            case RS2_OPTION_PROJECTOR_TEMPERATURE:
            case RS2_OPTION_ASIC_TEMPERATURE:
                field = &temperature::temperature;
                is_valid_field = &temperature::is_valid;
                break;
            default:
                throw invalid_value_exception(to_string() << _ep.get_option_name(_option) << " is not temperature option!");
        }

        if (0 == temperature_data.*is_valid_field)
            LOG_ERROR(_ep.get_option_name(_option) << " value is not valid!");

        return temperature_data.*field;
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

}