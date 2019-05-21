//
// Created by marko on 21.05.19..
//

#include "cs/cs-timestamp.h"

namespace librealsense {
    cs_timestamp_reader::cs_timestamp_reader(std::shared_ptr <platform::time_service> ts) :
            _ts(ts) {
        reset();
    }

    void cs_timestamp_reader::reset() {
        std::lock_guard <std::recursive_mutex> lock(_mtx);
        counter = 0;
    }

    rs2_time_t cs_timestamp_reader::get_frame_timestamp(const request_mapping &mode, const platform::frame_object &fo) {
        std::lock_guard <std::recursive_mutex> lock(_mtx);
        rs2_time_t time;

        if (fo.backend_time < 0) time = _ts->get_time();
        else time = fo.backend_time;

        return time;
    }

    unsigned long long
    cs_timestamp_reader::get_frame_counter(const request_mapping &mode, const platform::frame_object &fo) const {
        std::lock_guard <std::recursive_mutex> lock(_mtx);

        return ++counter;
        return 0;
    }

    rs2_timestamp_domain cs_timestamp_reader::get_frame_timestamp_domain(const request_mapping &mode,
                                                                         const platform::frame_object &fo) const {
        return RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME;
    }
}
