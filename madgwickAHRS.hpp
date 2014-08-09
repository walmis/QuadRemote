/*
 * madwigkAHRS.hpp
 *
 *  Created on: Apr 7, 2014
 *      Author: walmis
 */

#ifndef MADWIGKAHRS_HPP_
#define MADWIGKAHRS_HPP_

#include <xpcc/math.hpp>

extern volatile float beta;				// algorithm gain

//---------------------------------------------------------------------------------------------------
// Function declarations

void madgwickAHRSUpdate(xpcc::Vector3f gyro, xpcc::Vector3f accel,
		const xpcc::Vector3f mag, xpcc::Quaternion<float> &state);
void madgwickAHRSUpdateIMU(const xpcc::Vector3f gyro, const xpcc::Vector3f accel,
		xpcc::Quaternion<float> &state, float dt);


#endif /* MADWIGKAHRS_HPP_ */
