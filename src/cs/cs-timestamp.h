//
// Created by marko on 21.05.19..
//

#ifndef LIBREALSENSE2_CS_TIMESTAMP_H
#define LIBREALSENSE2_CS_TIMESTAMP_H

#include "sensor.h"

namespace librealsense
{
    class cs_timestamp_reader_from_metadata : public frame_timestamp_reader
    {
        std::unique_ptr<frame_timestamp_reader> _backup_timestamp_reader;
        std::atomic<bool> _has_metadata;
        bool one_time_note;
        mutable std::recursive_mutex _mtx;

    public:
        cs_timestamp_reader_from_metadata(std::unique_ptr<frame_timestamp_reader> backup_timestamp_reader);

        bool has_metadata(const request_mapping& mode, const void * metadata, size_t metadata_size);

        rs2_time_t get_frame_timestamp(const request_mapping& mode, const platform::frame_object& fo) override;

        unsigned long long get_frame_counter(const request_mapping & mode, const platform::frame_object& fo) const override;

        void reset() override;

        rs2_timestamp_domain get_frame_timestamp_domain(const request_mapping & mode, const platform::frame_object& fo) const override;
    };

    class cs_timestamp_reader : public frame_timestamp_reader
    {
    private:
        mutable int64_t counter;
        std::shared_ptr<platform::time_service> _ts;
        mutable std::recursive_mutex _mtx;

    public:
        cs_timestamp_reader(std::shared_ptr<platform::time_service> ts);

        void reset() override;

        rs2_time_t get_frame_timestamp(const request_mapping& mode, const platform::frame_object& fo) override;

        unsigned long long get_frame_counter(const request_mapping & mode, const platform::frame_object& fo) const override;

        rs2_timestamp_domain get_frame_timestamp_domain(const request_mapping & mode, const platform::frame_object& fo) const override;
    };
}

#endif //LIBREALSENSE2_CS_TIMESTAMP_H
