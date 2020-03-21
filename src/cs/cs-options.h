// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#pragma once

#include "ds5/ds5-private.h"

#include "backend.h"
#include "types.h"
#include "option.h"

namespace librealsense
{
    class cs_asic_and_projector_temperature_options : public readonly_option
    {
    public:
        float query() const override;

        option_range get_range() const override;

        bool is_enabled() const override;

        const char* get_description() const override;

        explicit cs_asic_and_projector_temperature_options(cs_sensor& ep, rs2_option opt, cs_stream stream);

    private:
        cs_sensor&                  _ep;
        rs2_option                  _option;
        cs_stream                   _stream;
    };

    class cs_depth_scale_option : public option, public observable_option
    {
    public:
        cs_depth_scale_option(hw_monitor& hwm);
        virtual ~cs_depth_scale_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Number of meters represented by a single depth unit";
        }
        void enable_recording(std::function<void(const option &)> record_action)
        {
            _record_action = record_action;
        }

    private:
        ds::depth_table_control get_depth_table(ds::advanced_query_mode mode) const;
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
    };

};