// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "firmware_logger_device.h"
#include <string>

namespace librealsense
{
    firmware_logger_device::firmware_logger_device(std::shared_ptr<hw_monitor> hardware_monitor) :
        _hw_monitor(hardware_monitor),
        _fw_logs(),
        _flash_logs(),
        _flash_logs_initialized(false),
        _parser(nullptr)
    {
        _input_code_for_fw_logs = { 0x14, 0x00, 0xab, 0xcd, 0x0f, 0x00, 0x00, 0x00,
                 0xf4, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

        _input_code_for_flash_logs = { 0x14, 0x00, 0xab, 0xcd, 0x09, 0x00, 0x00, 0x00,
                 0x00, 0xa0, 0x17, 0x00, 0xf8, 0x03, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    }

    bool firmware_logger_device::get_fw_log(fw_logs::fw_logs_binary_data& binary_data)
    {
        bool result = false;
        if (_fw_logs.empty())
        {
            get_fw_logs_from_hw_monitor();
        }

        if (!_fw_logs.empty())
        {
            fw_logs::fw_logs_binary_data data;
            data = _fw_logs.front();
            _fw_logs.pop();
            binary_data = data;
            result = true;
        }
        return result;
    }


    void firmware_logger_device::get_fw_logs_from_hw_monitor()
    {
        int size_of_fw_logs_header = 4;
        auto res = _hw_monitor->send(_input_code_for_fw_logs);
        if (res.empty())
        {
            LOG_INFO("Getting Firmware logs failed!");
            return;
        }

        //erasing header
        res.erase(res.begin(), res.begin() + size_of_fw_logs_header);

        auto beginOfLogIterator = res.begin();
        // convert bytes to fw_logs_binary_data
        for (int i = 0; i < res.size() / fw_logs::BINARY_DATA_SIZE; ++i)
        {
            if (*beginOfLogIterator == 0)
                break;
            auto endOfLogIterator = beginOfLogIterator + fw_logs::BINARY_DATA_SIZE;
            std::vector<uint8_t> resultsForOneLog;
            resultsForOneLog.insert(resultsForOneLog.begin(), beginOfLogIterator, endOfLogIterator);
            fw_logs::fw_logs_binary_data binary_data{ resultsForOneLog };
            _fw_logs.push(binary_data);
            beginOfLogIterator = endOfLogIterator;
        }
    }

    void firmware_logger_device::get_flash_logs_from_hw_monitor()
    {
        int size_of_flash_logs_header = 27;
        auto res = _hw_monitor->send(_input_code_for_flash_logs);
        if (res.empty())
        {
            LOG_INFO("Getting Flash logs failed!");
            return;
        }

        //erasing header
        res.erase(res.begin(), res.begin() + size_of_flash_logs_header);

        auto beginOfLogIterator = res.begin();
        // convert bytes to flash_logs_binary_data
        for (int i = 0; i < res.size() / fw_logs::BINARY_DATA_SIZE && *beginOfLogIterator == 160; ++i)
        {
            auto endOfLogIterator = beginOfLogIterator + fw_logs::BINARY_DATA_SIZE;
            std::vector<uint8_t> resultsForOneLog;
            resultsForOneLog.insert(resultsForOneLog.begin(), beginOfLogIterator, endOfLogIterator);
            fw_logs::fw_logs_binary_data binary_data{ resultsForOneLog };
            _flash_logs.push(binary_data);
            beginOfLogIterator = endOfLogIterator;
        }

        _flash_logs_initialized = true;
    }

    bool firmware_logger_device::get_flash_log(fw_logs::fw_logs_binary_data& binary_data)
    {
        bool result = false;
        if (!_flash_logs_initialized)
        {
            get_flash_logs_from_hw_monitor();
        }

        if (!_flash_logs.empty())
        {
            fw_logs::fw_logs_binary_data data;
            data = _flash_logs.front();
            _flash_logs.pop();
            binary_data = data;
            result = true;
        }
        return result;
    }

    bool firmware_logger_device::init_parser(std::string xml_content)
    {
        _parser = new fw_logs::fw_logs_parser(xml_content);

        return (_parser != nullptr);
    }

    bool firmware_logger_device::parse_log(const fw_logs::fw_logs_binary_data* fw_log_msg, 
        fw_logs::fw_log_data* parsed_msg)
	{
        bool result = false;
        if (_parser && parsed_msg && fw_log_msg)
        {
            *parsed_msg = _parser->parse_fw_log(fw_log_msg);
            result = true;
        }
            
        return result;
	}
}