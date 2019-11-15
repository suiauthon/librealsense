#include "cs/cs-factory.h"

namespace librealsense {
    std::shared_ptr <device_interface> cs_info::create(std::shared_ptr <context> ctx,
                                                       bool register_device_notifications) const {
        switch (get_camera_model(_hwm.id)) {
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
            case CS_D435E:
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
        smcs_api->FindAllDevices(0.15);
        auto devices = smcs_api->GetAllDevices();

        for (int i = 0; i < devices.size(); i++) {
            if (devices[i]->IsOnNetwork()) {
                auto info = platform::cs_device_info();
                info.serial = devices[i]->GetSerialNumber();
                info.id = devices[i]->GetModelName();
                info.info = devices[i]->GetManufacturerSpecificInfo();
                results.push_back(info);
            }
        }

        return results;
    }

    D435e_camera::D435e_camera(std::shared_ptr<context> ctx,
                               const platform::cs_device_info &hwm_device,
                               const platform::backend_device_group& group,
                               bool register_device_notifications)
            : device(ctx, group, register_device_notifications),
              cs_color(ctx, group, register_device_notifications),
              cs_depth(ctx, group, register_device_notifications),
              cs_advanced_mode_base()
    {
        _cs_device = ctx->get_backend().create_cs_device(hwm_device);

        _depth_device_idx = add_sensor(create_depth_device(ctx, _cs_device));
        _color_device_idx = add_sensor(create_color_device(ctx, _cs_device));

        depth_init(ctx, group);
        color_init(ctx, group);

        environment::get_instance().get_extrinsics_graph().register_extrinsics(*_color_stream, *_depth_stream, _color_extrinsic);

        register_info(RS2_CAMERA_INFO_NAME, "FRAMOS D435e"/*hwm_device.info*/);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, hwm_device.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, "0B07"/*hwm_device.id*/);
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, cs_depth::_fw_version);
        register_info(RS2_CAMERA_INFO_DEVICE_VERSION, _cs_device->get_device_version());

        cs_advanced_mode_init(cs_depth::_hw_monitor, &get_depth_sensor());
    }

    std::shared_ptr<matcher> D435e_camera::create_matcher(const frame_holder& frame) const
    {
        std::vector<stream_interface*> streams = {_color_stream.get(), _depth_stream.get(), _left_ir_stream.get() , _right_ir_stream.get()};
        if (frame.frame->supports_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER))
        {
            return matcher_factory::create(RS2_MATCHER_DLR_C, streams);
        }
        return matcher_factory::create(RS2_MATCHER_DEFAULT, streams);
    }

    std::vector<tagged_profile> D435e_camera::get_profiles_tags() const
    {
        std::vector<tagged_profile> markers;
        markers.push_back({ RS2_STREAM_DEPTH, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        markers.push_back({ RS2_STREAM_COLOR, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        //markers.push_back({ RS2_STREAM_INFRARED, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        return markers;
    }
}