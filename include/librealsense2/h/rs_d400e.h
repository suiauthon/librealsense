// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.


/** \file rs_d400e.h
* \brief Exposes functionality specific to D400e devices for C compilers
*/


#ifndef LIBREALSENSE_RS2_D400E_H
#define LIBREALSENSE_RS2_D400E_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_types.h"

/**
* Sets heartbeat time in seconds for D400e series devices.
* Heartbeat time is used for device disconnect detection.
* \param       time      New application heartbeat time in seconds for all D400e series devices. Heartbeat time on devices is set to 4x application heartbeat time
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_d400e_set_heartbeat_time(double time, rs2_error** error);

/**
* Acquires heartbeat time in seconds for D400e series devices.
* Heartbeat time is used for device disconnect detection.
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                Current application heartbeat time in seconds for all D400e devices. Heartbeat time on devices is 4x application heartbeat time
*/
double rs2_d400e_get_heartbeat_time(rs2_error** error);


#ifdef __cplusplus
}
#endif
#endif
