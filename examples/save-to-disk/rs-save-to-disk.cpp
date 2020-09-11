// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//
//#include <fstream>              // File IO
//#include <iostream>             // Terminal IO
//#include <sstream>              // Stringstreams
//#include <experimental/filesystem>
//#include <chrono>
//#include <ctime>
//#include <fstream>
//#include <iomanip>
//
//// 3rd party header for writing png files
//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "stb_image_write.h"
//
//// Helper function for writing metadata to disk as a csv file
//void metadata_to_csv(const rs2::frame& frm, const std::string& filename);
//
//
//bool SaveDataToDisk
//(
//    const char* dataIn,
//    uint32_t dataInSize,
//    bool printLog,
//    std::string fileName,
//    std::string filePath
//)
//{
//    //Framos_ASSERT(dataIn != NULL);
//    //Framos_ASSERT(dataInSize > 0);
//
//    bool status = false;
//    std::ostringstream ss;
//    std::string fPath = ".";
//    std::string fName = fileName;
//
//    if (!filePath.empty()) {
//        fPath = filePath;
//    }
//
//    // Add date and time to file name
//    auto now = std::chrono::system_clock::now();
//    auto time = std::chrono::system_clock::to_time_t(now);
//    std::put_time(std::localtime(&time), "%F_%H_%M_%S");
//    ss << std::put_time(std::localtime(&time), "%F_%H_%M_%S");
//    //fName += "_" + ss.str();
//
//    fPath += "\\" + fName;
//
//    std::ofstream fileOut(fPath.c_str(), std::fstream::out | std::fstream::binary);
//    if (fileOut.is_open()) {
//        fileOut.write(dataIn, dataInSize);
//        fileOut.close();
//        status = true;
//    }
//
//    return status;
//}
//
//
//
//// This sample captures 30 frames and writes the last frame to disk.
//// It can be useful for debugging an embedded system with no display.
//int main(int argc, char * argv[]) try
//{
//    // Declare depth colorizer for pretty visualization of depth data
//    rs2::colorizer color_map;
//
//    // Declare RealSense pipeline, encapsulating the actual device and sensors
//    rs2::pipeline pipe;
//
//    //RS2_FORMAT_RAW10, /**< Four 10 bits per pixel luminance values packed into a 5-byte macropixel */
//    //RS2_FORMAT_RAW16, /**< 16-bit raw image */
//    //RS2_FORMAT_RAW8, /**< 8-bit raw image */
//
//    rs2::config cfg;
//    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RAW16);
//
//    // Start streaming with default recommended configuration
//    pipe.start(cfg);
//
//    // Capture 30 frames to give autoexposure, etc. a chance to settle
//    for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();
//
//    // Wait for the next set of frames from the camera. Now that autoexposure, etc.
//    // has settled, we will write these to disk
//    for (auto&& frame : pipe.wait_for_frames())
//    {
//        // We can only save video frames as pngs, so we skip the rest
//        if (auto vf = frame.as<rs2::video_frame>())
//        {
//            auto stream = frame.get_profile().stream_type();
//            // Use the colorizer to get an rgb image for the depth stream
//            if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);
//
//            // Write images to disk
//            std::stringstream png_file;
//            png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
//            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(), vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
//            std::cout << "Saved " << png_file.str() << std::endl;
//
//            bool s =  SaveDataToDisk((const char*)(vf.get_data()), 1920*1080*2, false, "testic", ".");
//
//            // Record per-frame metadata for UVC streams
//            std::stringstream csv_file;
//            csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
//                     << "-metadata.csv";
//            metadata_to_csv(vf, csv_file.str());
//        }
//    }
//
//    return EXIT_SUCCESS;
//}
//catch(const rs2::error & e)
//{
//    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//    return EXIT_FAILURE;
//}
//catch(const std::exception & e)
//{
//    std::cerr << e.what() << std::endl;
//    return EXIT_FAILURE;
//}
//
//void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
//{
//    std::ofstream csv;
//
//    csv.open(filename);
//
//    //    std::cout << "Writing metadata to " << filename << endl;
//    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";
//
//    // Record all the available metadata attributes
//    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
//    {
//        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
//        {
//            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
//                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
//        }
//    }
//
//    csv.close();
//}




#include <fstream>
#include <librealsense2/rs.hpp>
#include "api.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main()
{
    rs2_error* e = nullptr;
    //int status = rs2_d400e_toggle_device_diagnostics("6CD146030D29", 1, &e);

    rs2::pipeline pipe;

    //rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RAW16);
    //cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8);

    //const rs2_device* device = pipe.get_active_profile().get_device().get().get();

    //pipe.start(cfg);
    pipe.start();

    rs2::device dev;


    int status = rs2_d400e_toggle_device_diagnostics("6CD146030D29", 0, &e);
    status = rs2_d400e_toggle_device_diagnostics("6CD146030D29", 0, &e);

    rs2_device* device;
    status = rs2_d400e_toggle_device_diagnostics("6CD146030D29", 1, &e);

    for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

    for (auto&& frame : pipe.wait_for_frames()) {

        if (auto vf = frame.as<rs2::video_frame>()) {

            // Store image in png format
            //std::stringstream png_file;
            //png_file << "raw_img" << vf.get_profile().stream_name() << ".png";
            //stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(), vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            // Other options: stbi_write_bmp, stbi_write_tga, stbi_write_hdr or stbi_write_jpg

            // Store image in raw-binary format
            std::string fPath = ".\\raw_imgColor";
            std::ofstream fileOut(fPath.c_str(), std::fstream::out | std::fstream::binary);
            if (fileOut.is_open()) {
                fileOut.write((const char*)(vf.get_data()), vf.get_width() * vf.get_height() * 2);
                fileOut.close();
            }

            break;
        }
    }
}
