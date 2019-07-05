// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.
#include "cs-hw-monitor.h"
#include "types.h"
#include <iomanip>

namespace librealsense
{

	void cs_hw_monitor::fill_usb_buffer(int opCodeNumber, int p1, int p2, int p3, int p4,
		uint8_t* data, int dataLength, uint8_t* bufferToSend, int& length)
	{
		auto preHeaderData = IVCAM_MONITOR_MAGIC_NUMBER;

		uint8_t* writePtr = bufferToSend;
		auto header_size = 4;

		auto cur_index = 2;
		memcpy(writePtr + cur_index, &preHeaderData, sizeof(uint16_t));
		cur_index += sizeof(uint16_t);
		memcpy(writePtr + cur_index, &opCodeNumber, sizeof(uint32_t));
		cur_index += sizeof(uint32_t);
		memcpy(writePtr + cur_index, &p1, sizeof(uint32_t));
		cur_index += sizeof(uint32_t);
		memcpy(writePtr + cur_index, &p2, sizeof(uint32_t));
		cur_index += sizeof(uint32_t);
		memcpy(writePtr + cur_index, &p3, sizeof(uint32_t));
		cur_index += sizeof(uint32_t);
		memcpy(writePtr + cur_index, &p4, sizeof(uint32_t));
		cur_index += sizeof(uint32_t);

		if (dataLength)
		{
			librealsense::copy(writePtr + cur_index, data, dataLength);
			cur_index += dataLength;
		}

		length = cur_index;
		uint16_t tmp_size = length - header_size;
		memcpy(bufferToSend, &tmp_size, sizeof(uint16_t)); // Length doesn't include header
	}

	void cs_hw_monitor::update_cmd_details(hwmon_cmd_details& details, size_t receivedCmdLen, unsigned char* outputBuffer)
	{
		details.receivedCommandDataLength = receivedCmdLen;

		if (details.oneDirection) return;

		if (details.receivedCommandDataLength < 4)
			throw invalid_value_exception("received incomplete response to usb command");

		details.receivedCommandDataLength -= 4;
		librealsense::copy(details.receivedOpcode.data(), outputBuffer, 4);

		if (details.receivedCommandDataLength > 0)
			librealsense::copy(details.receivedCommandData.data(), outputBuffer + 4, details.receivedCommandDataLength);
	}

	void cs_hw_monitor::send_hw_monitor_command(hwmon_cmd_details& details) const
	{
		unsigned char outputBuffer[HW_MONITOR_BUFFER_SIZE];

		uint32_t op{};
		size_t receivedCmdLen = HW_MONITOR_BUFFER_SIZE;

		execute_usb_command(details.sendCommandData.data(), details.sizeOfSendCommandData, op, outputBuffer, receivedCmdLen);
		update_cmd_details(details, receivedCmdLen, outputBuffer);
	}

	void cs_hw_monitor::execute_usb_command(uint8_t *out, size_t outSize, uint32_t & op, uint8_t * in, size_t & inSize) const
	{
		std::vector<uint8_t> out_vec(out, out + outSize);

		UINT64 address, regLength;
		GEV_STATUS gevStatus;
		double maxWaitTime = 0.2;

		uint32_t reg = 0;
		uint16_t SCPS0_ADDR = 0x0D04;
		uint16_t SCPD0_ADDR = 0x0D08;
		uint16_t SCPS1_ADDR = 0x0D44;
		uint16_t SCPD1_ADDR = 0x0D48;
		uint16_t PKT_SIZE = 7996;
		uint16_t PKT_DLY = 10;
				
		auto res = send(out_vec);

		inSize = res.size();
		librealsense::copy(in, res.data(), inSize);
		
		//platform::cs_device::_hwm_device->SetMemory(address, 512, (UINT8*)out_vec.data, &gevStatus, maxWaitTime);

		//platform::cs_device::_connected_device->CommandNodeExecute("STR_HWmTxBufferSend");


		//auto res = _locked_transfer->send_receive(out_vec);
	}

	std::vector<uint8_t> cs_hw_monitor::send(std::vector<uint8_t> data) const
	{
        return static_cast<std::vector<uint8_t>>(_ep.invoke_powered(
                [this, &data](platform::cs_device& dev)
                {
                    return dev.send_hwm(data);
                }));
	}

	std::vector<uint8_t> cs_hw_monitor::send(command cmd) const
	{
		hwmon_cmd newCommand(cmd);
		auto opCodeXmit = static_cast<uint32_t>(newCommand.cmd);

		hwmon_cmd_details details;
		details.oneDirection = newCommand.oneDirection;
		details.timeOut = newCommand.timeOut;

		fill_usb_buffer(opCodeXmit,
			newCommand.param1,
			newCommand.param2,
			newCommand.param3,
			newCommand.param4,
			newCommand.data,
			newCommand.sizeOfSendCommandData,
			details.sendCommandData.data(),
			details.sizeOfSendCommandData);

		send_hw_monitor_command(details);

		// Error/exit conditions
		if (newCommand.oneDirection)
			return std::vector<uint8_t>();

		librealsense::copy(newCommand.receivedOpcode, details.receivedOpcode.data(), 4);
		librealsense::copy(newCommand.receivedCommandData, details.receivedCommandData.data(), details.receivedCommandDataLength);
		newCommand.receivedCommandDataLength = details.receivedCommandDataLength;

		// endian?
		auto opCodeAsUint32 = pack(details.receivedOpcode[3], details.receivedOpcode[2],
			details.receivedOpcode[1], details.receivedOpcode[0]);
		if (opCodeAsUint32 != opCodeXmit)
		{
			auto err_type = static_cast<hwmon_response>(opCodeAsUint32);
			throw invalid_value_exception(to_string() << "hwmon command 0x" << std::hex << opCodeXmit << " failed. Error type: "
				<< hwmon_error2str(err_type) << " (" << std::dec << (int)err_type << ").");
		}

		return std::vector<uint8_t>(newCommand.receivedCommandData,
			newCommand.receivedCommandData + newCommand.receivedCommandDataLength);
	}

	void cs_hw_monitor::get_gvd(size_t sz, unsigned char* gvd, uint8_t gvd_cmd) const
	{
		command command(gvd_cmd);
		auto data = send(command);
		auto minSize = std::min(sz, data.size());
		librealsense::copy(gvd, data.data(), minSize);
	}

	std::string cs_hw_monitor::get_firmware_version_string(int gvd_cmd, uint32_t offset) const
	{
		std::vector<unsigned char> gvd(HW_MONITOR_BUFFER_SIZE);
		get_gvd(gvd.size(), gvd.data(), gvd_cmd);
		uint8_t fws[8];
		librealsense::copy(fws, gvd.data() + offset, 8);
		return to_string() << static_cast<int>(fws[3]) << "." << static_cast<int>(fws[2])
			<< "." << static_cast<int>(fws[1]) << "." << static_cast<int>(fws[0]);
	}

	std::string cs_hw_monitor::get_module_serial_string(uint8_t gvd_cmd, uint32_t offset, int size) const
	{
		std::vector<unsigned char> gvd(HW_MONITOR_BUFFER_SIZE);
		get_gvd(gvd.size(), gvd.data(), gvd_cmd);
		unsigned char ss[8];
		librealsense::copy(ss, gvd.data() + offset, 8);
		std::stringstream formattedBuffer;
		for (auto i = 0; i < size; i++)
		{
			formattedBuffer << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(ss[i]);
		}
		return formattedBuffer.str();
	}

	bool cs_hw_monitor::is_camera_locked(uint8_t gvd_cmd, uint32_t offset) const
	{
		std::vector<unsigned char> gvd(HW_MONITOR_BUFFER_SIZE);
		get_gvd(gvd.size(), gvd.data(), gvd_cmd);
		bool value;
		librealsense::copy(&value, gvd.data() + offset, 1);
		return value;
	}
}
