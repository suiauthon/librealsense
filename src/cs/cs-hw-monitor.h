// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include "sensor.h"
#include "cs-factory.h"
#include "hw-monitor.h"
#include <mutex>
using namespace librealsense::platform;

namespace librealsense
{
	class cs_hw_monitor
	{
        struct hwmon_cmd
        {
            uint8_t     cmd;
            int         param1;
            int         param2;
            int         param3;
            int         param4;
            uint8_t     data[HW_MONITOR_BUFFER_SIZE];
            int         sizeOfSendCommandData;
            long        timeOut;
            bool        oneDirection;
            uint8_t     receivedCommandData[HW_MONITOR_BUFFER_SIZE];
            size_t      receivedCommandDataLength;
            uint8_t     receivedOpcode[4];

            explicit hwmon_cmd(uint8_t cmd_id)
                    : cmd(cmd_id),
                      param1(0),
                      param2(0),
                      param3(0),
                      param4(0),
                      sizeOfSendCommandData(0),
                      timeOut(5000),
                      oneDirection(false),
                      receivedCommandDataLength(0)
            {}


            explicit hwmon_cmd(const command& cmd)
                    : cmd(cmd.cmd),
                      param1(cmd.param1),
                      param2(cmd.param2),
                      param3(cmd.param3),
                      param4(cmd.param4),
                      sizeOfSendCommandData(std::min((uint16_t)cmd.data.size(), HW_MONITOR_BUFFER_SIZE)),
                      timeOut(cmd.timeout_ms),
                      oneDirection(!cmd.require_response),
                      receivedCommandDataLength(0)
            {
                librealsense::copy(data, cmd.data.data(), sizeOfSendCommandData);
            }
        };

        struct hwmon_cmd_details
        {
            bool                                         oneDirection;
            std::array<uint8_t, HW_MONITOR_COMMAND_SIZE> sendCommandData;
            int                                          sizeOfSendCommandData;
            long                                         timeOut;
            std::array<uint8_t, 4>                       receivedOpcode;
            std::array<uint8_t, HW_MONITOR_BUFFER_SIZE>  receivedCommandData;
            size_t                                       receivedCommandDataLength;
        };

        static void fill_usb_buffer(int opCodeNumber, int p1, int p2, int p3, int p4, uint8_t* data, int dataLength, uint8_t* bufferToSend, int& length);
        void execute_usb_command(uint8_t *out, size_t outSize, uint32_t& op, uint8_t* in, size_t& inSize) const;
        static void update_cmd_details(hwmon_cmd_details& details, size_t receivedCmdLen, unsigned char* outputBuffer);
        void send_hw_monitor_command(hwmon_cmd_details& details) const;

        cs_sensor& _ep;
	public:
        explicit cs_hw_monitor(cs_sensor& ep)
                : _ep(ep)
        {}

        std::vector<uint8_t> send(std::vector<uint8_t> data) const;
        std::vector<uint8_t> send(command cmd) const;
        void get_gvd(size_t sz, unsigned char* gvd, uint8_t gvd_cmd) const;
        std::string get_firmware_version_string(int gvd_cmd, uint32_t offset) const;
        std::string get_module_serial_string(uint8_t gvd_cmd, uint32_t offset, int size = 6) const;
        bool is_camera_locked(uint8_t gvd_cmd, uint32_t offset) const;
	};
}


