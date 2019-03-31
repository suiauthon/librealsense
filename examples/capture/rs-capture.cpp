// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    printf("Nakon konstruktora\n");

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    pipe.start();

    printf("Nakon starta\n");

    while (app) // Application still alive?
    {
        printf("\n");
        rs2::frameset data = pipe.wait_for_frames();//.    // Wait for next set of frames from the camera
                             //apply_filter(printer).     // Print each enabled stream frame rate
                             //apply_filter(color_map);   // Find and colorize the depth data
        printf("Size: %d\n", data.size());
        printf("Frame number: %d\n", data.get_frame_number());
        printf("Time stamp: %f\n", data.get_timestamp());
        uint8_t *pointer_to_int;
        pointer_to_int = (uint8_t *) data.get_data();
        printf("Frame data %d\n", pointer_to_int[0]);
        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
        printf("Pokazi ga\n");
        app.show(data);
        printf("Nakon pokazivanja\n");
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}