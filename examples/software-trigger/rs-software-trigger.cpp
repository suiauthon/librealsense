// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering
#include <thread>
#include <map>
#include <vector>
float getOptimalInterPacketDelay(int num_parallel_streams, int packetSize);

using namespace rs2;

int main(int argc, char* argv[]) try
{
    // Create a simple OpenGL window for rendering:
    //window app(1920, 1080, "CPP Multi-Camera Example");

    rs2::context                ctx;            // Create librealsense context for managing devices

    rs2::colorizer              colorizer;      // Utility class to convert depth data RGB colorspace

                                                //std::vector<rs2::pipeline>  pipelines;
    rs2::syncer sync;

    std::vector<rs2::sensor> sensors;
    std::vector<rs2::sensor> sensors1;
    std::vector<rs2::stream_profile> sensor_stream_profiles;

    for (auto&& dev : ctx.query_devices()) {
        std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
        if (name.compare("FRAMOS D415e") == 0) {
            for (auto&& sensor : dev.query_sensors()) {
                // adjust InterPacketDelay option on D435e camera based on PacketSize, number of cameras and number of streams
                // assumptions: 
                //  - Two D435e cameras are streaming to single NIC on PC
                //  - only depth and color streams are enabled on all cameras
                //  - PacketSize is the same on all cameras
                int numParallelStreams = 1; // (1 cameras * 2 streams) - 1
                                            //float packetSize = 7996;  // Manual - select this value if jumbo frames on NIC are enabled
                                            //float packetSize = 1500;  // Manual - select this value if jumbo frames on NIC are disabled
                float packetSize = sensor.get_option(RS2_OPTION_PACKET_SIZE);   // Automatic packet size discovery
                float interPacketDelay = getOptimalInterPacketDelay(numParallelStreams, packetSize);

                std::cout << sensor.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ", Packet Size = " << packetSize << " InterPacketDelay = " << interPacketDelay << std::endl;
                sensor.set_option(RS2_OPTION_PACKET_SIZE, packetSize);
                sensor.set_option(RS2_OPTION_INTER_PACKET_DELAY, interPacketDelay);
                //sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
                //sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
            }
        }
    }

    int min = 1;
    int max = 60;

    int min_ir = 1;
    int max_ir = 68;

    // Start a streaming pipe per each connected device
    for (auto&& dev : ctx.query_devices())
    {
        sensors1 = dev.query_sensors();
        auto bla = dev.query_sensors();
        for (rs2::sensor sensor : dev.query_sensors()) {
            sensor_stream_profiles = sensor.get_stream_profiles();
            srand(clock());
            auto v1 = rand() % (max - min + 1) + min;
            auto profile = sensor_stream_profiles[v1];
            if (auto depth_sensor = sensor.as<rs2::color_sensor>()) {
                depth_sensor.open(profile); // 640 x 480 60Hz
                depth_sensor.start(sync);
                sensors.push_back(sensor);
            }
        }
    }

    // We'll keep track for the last frame of each stream available to make the presentation persistent
    std::map<int, rs2::frame> render_frames;


    clock_t endwait;
    endwait = clock() + 5 * CLOCKS_PER_SEC;
    int cnt = 0;
    // Main app loop
    while (1) {
        endwait = clock() + 4 * CLOCKS_PER_SEC;
        while (true)
            //while (app)
        {
            // Collect the new frames from all the connected devices
            std::vector<rs2::frame> new_frames;
            //for (auto &&pipe : pipelines)
            {
                rs2::frameset fs;
                if (sync.poll_for_frames(&fs))
                {
                    for (const rs2::frame& f : fs)
                        new_frames.emplace_back(f);
                }
            }

            // Convert the newly-arrived frames to render-firendly format
            for (const auto& frame : new_frames)
            {
                auto format = frame.get_profile().format();
                if (format == RS2_FORMAT_RGB8)
                    render_frames[frame.get_profile().unique_id()] = frame;
            }

            // Present all the collected frames with openGl mosaic

            //app.show(render_frames);
            if (clock() > endwait)
                break;
        }
        cnt++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (sensors.size() > 1) {
            sensors.pop_back();
        }
        for (auto sensor : sensors) {
            if (auto depth_sensor = sensor.as<rs2::color_sensor>()) {
                sensor.stop();
                sensor.close();

            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (rs2::sensor sensor : sensors1) {
            sensor_stream_profiles = sensor.get_stream_profiles();

            if (auto depth_sensor = sensor.as<rs2::color_sensor>()) {
                srand(clock());

                auto v1 = rand() % (max - min + 1) + min;
                auto profile = sensor_stream_profiles[v1];
                depth_sensor.open(profile); // 640 x 480 60Hz
                depth_sensor.start(sync);
                sensors.insert(sensors.begin(), sensor);

                if (auto vp = profile.as<video_stream_profile>())
                {
                    std::cout << "Resolution: " << vp.width() << "x" << vp.height() << " @ " << vp.fps() << "FPS" << " test: " << cnt << std::endl;
                }
            }
        }


    }
    /*if (auto depth_sensor = sensors1[0].as<rs2::depth_sensor>()) {
    depth_sensor.stop();
    depth_sensor.close();

    auto v1 = rand() % 100;
    depth_sensor.open(sensor_stream_profiles[v1]); // 640 x 480 60Hz
    depth_sensor.start(sync);
    sensors.push_back(sensors1[0]);
    }*/



    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    while (1);
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    while (1);
    return EXIT_FAILURE;
}

// calculate optimal InterPacketDelay for D435e camera based on PacketSize and number of parallel streams
float getOptimalInterPacketDelay(int num_parallel_streams, int packetSize)
{
    float interPacketDelay = 0;
    float ethPacketSize = packetSize + 38;  // 38 bytes overhead
    float nsPerByte = 8.0;  // for 1Gbps

    float timeToTransferPacket = (ethPacketSize * nsPerByte) / 1000.0;  // time in us
    timeToTransferPacket = ceil(timeToTransferPacket + 0.5);            // round up
    interPacketDelay = timeToTransferPacket * num_parallel_streams;

    return interPacketDelay;
}