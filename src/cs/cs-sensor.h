// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#ifndef LIBREALSENSE2_CS_SENSOR_H
#define LIBREALSENSE2_CS_SENSOR_H

#include <smcs_cpp/CameraSDK.h>
#include <smcs_cpp/IImageBitmap.h>
#include "sensor.h"
#include "types.h"
#include <unordered_map>

namespace librealsense {

    const std::string CS_CAMERA_MODEL_D435e = "D435e";
    const std::string CS_CAMERA_MODEL_D415e = "D415e";

    typedef enum cs_stream {
        CS_STREAM_DEPTH = 0,
        CS_STREAM_COLOR,
        CS_STREAM_IR_LEFT,
        CS_STREAM_IR_RIGHT,
        CS_STREAM_COUNT
    } cs_stream;

    enum cs_camera_model {
        CS_D435E,
        CS_D415E,
        CS_UNDEFINED
    };

    typedef enum cs_inter_cam_sync_mode {
        CS_INTERCAM_SYNC_DEFAULT,
        CS_INTERCAM_SYNC_MASTER,
        CS_INTERCAM_SYNC_SLAVE,
        CS_INTERCAM_SYNC_EXTERNAL,
        CS_INTERCAM_SYNC_MAX
    } cs_inter_cam_mode;

    typedef enum cs_inter_cam_sync_mode_color {
        CS_INTERCAM_SYNC_DEFAULT_COLOR,
        CS_INTERCAM_SYNC_EXTERNAL_COLOR,
        CS_INTERCAM_SYNC_MAX_COLOR
    } cs_inter_cam_mode_color;

    typedef enum cs_user_output_level {
        CS_USER_OUTPUT_LEVEL_LOW = 1,
        CS_USER_OUTPUT_LEVEL_HIGH,
        CS_USER_OUTPUT_LEVEL_MAX
    } cs_user_output_level;

    class cs_firmware_version
    {
    public:
        explicit cs_firmware_version(UINT32 major = 0, UINT32 minor = 0, UINT32 patch = 0, UINT32 build = 0)
            : _major(major)
            , _minor(minor)
            , _patch(patch)
            , _build(build)
        {}

        explicit cs_firmware_version(smcs::IDevice &device);

        bool operator > (const cs_firmware_version &other)
        {
            if (_major > other._major)
                return true;
            else if (_major < other._major)
                return false;

            if (_minor > other._minor)
                return true;
            else if (_minor < other._minor)
                return false;

            if (_patch > other._patch)
                return true;
            else if (_patch < other._patch)
                return false;

            if (_build > other._build)
                return true;
            else if (_build < other._build)
                return false;

            return false;
        }

        bool operator < (const cs_firmware_version &other)
        {
            return !(*this >= other);
        }

        bool operator == (const cs_firmware_version &other)
        {
            return
                (_major == other._major) &&
                (_minor == other._minor) &&
                (_patch == other._patch) &&
                (_build == other._build);
        }

        bool operator >= (const cs_firmware_version &other)
        {
            return (*this > other) || (*this == other);
        }

        bool operator <= (const cs_firmware_version &other)
        {
            return (*this < other) || (*this == other);
        }

    private:
        UINT32 _major, _minor, _patch, _build;
    };

    namespace platform {
        class cs_device {
        public:
            explicit cs_device(platform::cs_device_info hwm);
            ~cs_device();

            power_state set_power_state(power_state state);

            void stream_on(std::function<void(const notification &n)> error_handler, std::vector<cs_stream> streams, std::vector<platform::stream_profile> profiles);

            void probe_and_commit(stream_profile profile, frame_callback callback, cs_stream stream);

            void close(std::vector<cs_stream> streams);

            void image_poll(cs_stream stream, UINT32 channel);

            power_state get_power_state() const { return _power_state; }

            bool get_pu(rs2_option opt, int32_t &value, cs_stream stream);

            bool set_pu(rs2_option opt, int32_t value, cs_stream stream);

            control_range get_pu_range(rs2_option option, cs_stream stream);

            std::vector <stream_profile> get_profiles(cs_stream stream);

            bool reset(void);

            std::vector<byte> send_hwm(std::vector<byte>& buffer);

            std::string get_device_version();
            std::string get_ip_address();
            std::string get_subnet_mask();

            enum rs2_format get_rgb_format();
            bool is_infrared_supported();
            bool is_option_supported(rs2_option opt, cs_stream stream);
            bool is_software_trigger_supported();
            bool is_line_debouncer_time_supported();
            bool is_rgb_raw16_supported();
            double get_device_timestamp_ms();

            void set_trigger_mode(float mode, cs_stream stream);
            float get_trigger_mode(cs_stream stream);
            bool get_intercam_mode(cs_stream stream);
            void update_external_trigger_mode_flag(cs_stream stream, float value);

            void toggle_raw16_flag(bool toggle) { _is_raw16 = toggle; };

        protected:
            void capture_loop(cs_stream stream, UINT32 channel);

            void set_format(stream_profile profile, cs_stream stream);

            std::vector<std::function<void(const notification &n)>> _error_handler;
            std::vector<stream_profile> _profiles;

        private:

            std::string get_cs_param_name(rs2_option option, cs_stream stream);

            bool get_cs_param_min(rs2_option option, int32_t &value, cs_stream stream);

            bool get_cs_param_max(rs2_option option, int32_t &value, cs_stream stream);

            bool get_cs_param_step(rs2_option option, int32_t &value, cs_stream stream);

            bool get_cs_param_value(rs2_option option, int32_t &value, cs_stream stream);

            bool set_cs_param(rs2_option option, int32_t value, cs_stream stream);

            int32_t round_cs_param(rs2_option option, int32_t value, cs_stream stream);

            void init_stream(std::function<void(const notification& n)> error_handler, cs_stream stream);

            void deinit_stream(cs_stream stream);

            void start_acquisition(cs_stream stream);

            void stop_acquisition(cs_stream stream);

            bool select_source(cs_stream stream);
            bool set_source_locked(cs_stream stream, bool locked);
            bool set_region(cs_stream stream, bool enable);
            bool disable_source_regions(cs_stream stream);
            bool select_region(cs_stream stream);
            bool select_channel(cs_stream stream);

            INT64 get_stream_source(cs_stream stream);
            INT64 get_stream_region(cs_stream stream);
            bool get_stream_channel(cs_stream stream, UINT32& channel);
            std::vector<cs_stream> get_stream_group(cs_stream stream);

            void stream_params_lock(cs_stream stream);
            void stream_params_unlock(cs_stream stream);

            uint32_t read_from_buffer(std::vector<byte>& buffer, uint32_t index);

            std::vector<byte> send_hwm_to_device(std::vector<byte>& buffer);
            void set_rgb_ae_roi(uint32_t top, uint32_t left, uint32_t bottom, uint32_t right);

            std::vector<uint32_t> get_frame_rates(); 
            bool get_frame_rates_from_control(std::vector<uint32_t> &frame_rates);

            bool is_profile_format(const smcs::IImageInfo& image_info, const stream_profile& profile);

            uint32_t cs_pixel_format_to_native_pixel_format(std::string cs_format); 
            uint32_t native_pixel_format_to_cs_pixel_format(uint32_t native_format);

            std::string ip_address_to_string(uint32_t ip_address);

            INT64 get_optimal_inter_packet_delay(INT64 packet_size_bytes, INT64 link_speed_mbitps);

            static bool inc_device_count_sn(std::string serialNum);
            static bool dec_device_count_sn(std::string serialNum);
            static int get_device_count_sn(std::string serialNum);
            static bool set_device_init_flag_sn(std::string serialNum, bool setInitFlag);
            static bool get_device_init_flag_sn(std::string serialNum);
            static bool set_device_option_sw_trigger_all_flag_sn(std::string serialNum, bool setTriggerAllFlag);
            static bool get_device_option_sw_trigger_all_flag_sn(std::string serialNum);

            bool is_temperature_supported();

            // members
            std::vector<std::unique_ptr <std::thread>> _threads;
            platform::cs_device_info _device_info;
            power_state _power_state;
            std::vector<std::atomic<bool>> _is_capturing;
            std::mutex _power_lock, _stream_lock, _hwm_lock;
            uint8_t _number_of_streams;
            smcs::ICameraAPI _smcs_api;
            smcs::IDevice _connected_device;
            std::unordered_map<cs_stream, UINT32, std::hash<int>> _stream_channels;
            std::vector<frame_callback> _callbacks;
            cs_firmware_version _cs_firmware_version;
            metadata_framos_basic _md;
            enum rs2_format _rgb_pixel_format;
            bool _infrared_supported;
            bool _temperature_supported_checked;
            bool _temperature_supported;
            bool _software_trigger_supported_checked;
            bool _software_trigger_supported;
            bool _line_debouncer_time_supported_checked;
            bool _line_debouncer_time_supported;
            bool _rgb_raw16_supported_checked;
            bool _rgb_raw16_supported;
            INT64 _selected_source;
            bool _selected_source_initialized;
            static std::map<std::string, int> _cs_device_num_objects_sn; // serial_number, number of objects per SN (device creation)
            static std::map<std::string, bool> _cs_device_initialized_sn; // serial_number, is device with SN initialized
            static std::map<std::string, bool> _cs_device_option_sw_trigger_all_flag_sn; // serial_number, is device with SN initialized

            double _timestamp_to_ms_factor;

            bool _is_raw16;
        };
    }

    class cs_sensor : public sensor_base
    {
    public:
        explicit cs_sensor(std::string name, std::shared_ptr<platform::cs_device> cs_device,
            std::unique_ptr<frame_timestamp_reader> timestamp_reader, device* dev, cs_stream stream);
        virtual ~cs_sensor() override;

        rs2_extension stream_to_frame_types(rs2_stream stream);

        virtual stream_profiles init_stream_profiles() override;

        void open(const stream_profiles& requests) override;
        void close() override;
        void start(frame_callback_ptr callback) override;
        void stop() override;

        template<class T>
        auto invoke_powered(T action)
        -> decltype(action(*static_cast<platform::cs_device*>(nullptr)))
        {
            power on(std::dynamic_pointer_cast<cs_sensor>(shared_from_this()));
            return action(*_device);
        }

        std::vector<platform::stream_profile> get_configuration() const { return _internal_config; }

        void register_pu(rs2_option id);

        void try_register_pu(rs2_option id);

        void set_inter_cam_sync_mode(float value);
        float get_inter_cam_sync_mode();
        bool query_inter_cam_sync_mode();

    private:
        void acquire_power();

        void release_power();

        void reset_streaming();

        cs_stream get_stream(rs2_stream type, int index);

        struct power
        {
            explicit power(std::weak_ptr<cs_sensor> owner)
                    : _owner(owner)
            {
                auto strong = _owner.lock();
                if (strong)
                {
                    strong->acquire_power();
                }
            }

            ~power()
            {
                if (auto strong = _owner.lock())
                {
                    try
                    {
                        strong->release_power();
                    }
                    catch (...) {}
                }
            }
        private:
            std::weak_ptr<cs_sensor> _owner;
        };

        std::unique_ptr<frame_timestamp_reader> _timestamp_reader;
        std::atomic<int> _user_count;
        std::mutex _power_lock;
        std::mutex _configure_lock;
        cs_stream _cs_stream;
        std::vector<cs_stream> _cs_selected_streams;
        std::unique_ptr<power> _power;
        std::shared_ptr<platform::cs_device> _device;
        std::atomic<bool> _external_trigger_mode;
    };

    class cs_command_transfer : public platform::command_transfer
    {
    public:
        std::vector<uint8_t> send_receive(const std::vector<uint8_t>& data, int, bool require_response) override;

        cs_command_transfer(std::shared_ptr<platform::cs_device> device)
            : _device(std::move(device))
        {};

    private:
        std::shared_ptr<platform::cs_device> _device;
    };
}

#endif //LIBREALSENSE2_CS_SENSOR_H
