/** \file cs_option.h
* \brief
* Exposes sensor options functionality for C compilers
*/

#ifndef LIBREALSENSE2_CS_OPTION_H
#define LIBREALSENSE2_CS_OPTION_H


#ifdef __cplusplus
extern "C" {
#endif

/** \brief Defines general configuration controls.
These can generally be mapped to camera controls, and unless stated otherwise, can be set/queried at any time.
*/
typedef enum cs_option
{
    CS_OPTION_EXPOSURE, /**< Controls exposure time of color camera. Setting any value will disable auto exposure*/
} cs_option;

#ifdef __cplusplus
}
#endif

#endif //LIBREALSENSE2_CS_OPTION_H
