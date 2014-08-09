/*
 * SensorProcessor.hpp
 *
 *  Created on: Apr 10, 2014
 *      Author: walmis
 */

#ifndef SENSORPROCESSOR_HPP_
#define SENSORPROCESSOR_HPP_

#include <xpcc/architecture.hpp>
#include <math.h>

#include "MMA8451Q.hpp"
#include "L3GD20.hpp"
#include "madgwickAHRS.hpp"

using namespace xpcc;

template<typename Filter>
class VecFilter {
	Filter x;
	Filter y;
	Filter z;

public:
	void append(xpcc::Vector3f &vec) {
		x.append(vec[0]);
		y.append(vec[1]);
		z.append(vec[2]);

	}

	void getValue(xpcc::Vector3f &vec) {
		vec[0] = x.getValue();
		vec[1] = y.getValue();
		vec[2] = z.getValue();
	}
};

class Median3Vec {
	xpcc::filter::Median<float, 3> x;
	xpcc::filter::Median<float, 3> y;
	xpcc::filter::Median<float, 3> z;

public:
	void push(xpcc::Vector3f &vec) {
		x.append(vec[0]);
		y.append(vec[1]);
		z.append(vec[2]);

		x.update();
		y.update();
		z.update();
	}

	void getValue(xpcc::Vector3f &vec) {
		vec[0] = x.getValue();
		vec[1] = y.getValue();
		vec[2] = z.getValue();
	}
};

class SensorProcessor : TickerTask {
public:

	xpcc::Vector3f vGyro; //gyroscope vector
	xpcc::Vector3f vAcc; //accelerometer vector
	xpcc::Vector3f vMag; //magnetometer vector

	xpcc::Vector3f dynamicAcc; //dynamic acceleration
	xpcc::Vector3f velocity; //estimated velocity

	xpcc::Quaternion<float> qRotation;

	xpcc::Quaternion<float> qRotationOffset;

	SensorProcessor() : qRotation(1.0, 0, 0, 0),
			qRotationOffset(1.0, 0, 0, 0)
	{

		zero();
	}

	void initialize() {
		xpcc::delay_ms(50);

		accel.initialize(0x1C);
		gyro.initialize(0x6A);
	}

	xpcc::Vector3f getEulerRad() {
	  xpcc::Vector3f angles;

	  xpcc::Quaternion<float> &q = qRotation;

	  angles[0] = atan2f(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
	  angles[1] = -asinf(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
	  angles[2] = atan2f(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
	  return angles;
	}

	xpcc::Vector3f getEuler() {
		return getEulerRad() * 180.0/M_PI;
	}

	xpcc::Vector3f getYawPitchRollRad() {
		xpcc::Vector3f angles;

		float gx, gy, gz; // estimated gravity direction

		xpcc::Quaternion<float> &q = qRotation;

		gx = 2 * (q[1]*q[3] - q[0]*q[2]);
		gy = 2 * (q[0]*q[1] + q[2]*q[3]);
		gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

		angles[0] = atan2f(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
		angles[1] = atanf(gx / sqrtf(gy*gy + gz*gz));
		angles[2] = atanf(gy / sqrtf(gx*gx + gz*gz));

//        float rotateXa0 = 2.0*(q.y*q.z + q.w*q.x);
//        float rotateXa1 = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
//        float rotateX = 0.0;
//        if (rotateXa0 != 0.0 && rotateXa1 != 0.0)
//            rotateX = atan2f(rotateXa0, rotateXa1);
//
//        float rotateYa0 = -2.0*(q.x*q.z - q.w*q.y);
//        float rotateY = 0.0;
//        if( rotateYa0 >= 1.0 )
//            rotateY = M_PI/2.0;
//        else if( rotateYa0 <= -1.0 )
//            rotateY = -M_PI/2.0;
//        else rotateY = asinf(rotateYa0);
//
//        float rotateZa0 = 2.0*(q.x*q.y + q.w*q.z);
//        float rotateZa1 = q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z;
//        float rotateZ = 0.0;
//        if (rotateZa0 != 0.0 && rotateZa1 != 0.0)
//            rotateZ = atan2f(rotateZa0, rotateZa1);
//
//
//        //yaw
//        angles[0] = rotateZ;
//        //pitch
//        angles[1] = rotateX;
//        //roll
//        angles[2] = rotateY;

		return angles;
	}


	void zero() {
		gyroCalib.restart(2000);
		gyroOffset = 0;
		qRotationOffset = Quaternion<float>(1.0,0,0,0);
		numSamples = 0;
	}

	xpcc::Vector3f getYawPitchRoll() {
		return getYawPitchRollRad() * 180.0/M_PI;
	}

	Vector3f gravityCompensateAcc(const xpcc::Vector3f &acc, const xpcc::Quaternion<float> &q) {
	  float g[3];

	  // get expected direction of gravity in the sensor frame
	  g[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
	  g[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
	  g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	  Vector3f vec;
	  // compensate accelerometer readings with the expected direction of gravity
	  vec[0] = acc[0] - g[0];
	  vec[1] = acc[1] - g[1];
	  vec[2] = acc[2] - g[2];

	  return vec;
	}

	void handleTick() {
		static PeriodicTimer<> t(100);
		static PeriodicTimer<> tRead(5);

		if(accel.isDataAvail()) {
			accel.getXYZ(vAcc);
		}

		if(gyro.isDataAvail()) {
			gyro.getXYZ(vGyro);

			vGyro.x *= -1.0;
			vGyro.y *= -1.0;

			vGyro *= M_PI/180.0;
		}

		static uint8_t read = 0;

		if(tRead.isExpired()) {
			read = 0;

			if(gyroCalib.isExpired() && gyroCalib.isActive()) {
				gyroOffset /= numSamples;
				accZero /= numSamples;
				XPCC_LOG_DEBUG .printf("calibration complete\n");
				XPCC_LOG_DEBUG << "Gyro offset:" << gyroOffset << "\n";
				XPCC_LOG_DEBUG << "Acc zero value: " << accZero << "\n";
				gyroCalib.stop();

				auto g = Vector3f(0,0,1);

				auto cross = accZero.cross(g);

				qRotationOffset = Quaternion<float>(sqrtf(accZero.getLengthSquared() * g.getLengthSquared()) + accZero * g,
						cross.x, cross.y, cross.z);

				qRotationOffset.normalize();
				XPCC_LOG_DEBUG << "Q - " << qRotationOffset << "\n";

				//reset rotation
				qRotation = xpcc::Quaternion<float>(1.0, 0, 0, 0);

			}
			if(gyroCalib.isActive()) {
				gyroOffset += vGyro;
				accZero += vAcc;
				numSamples++;
			} else {
				//if calibrated
				//remove offset from gyro readings
				vGyro -= gyroOffset;
			}

			//XPCC_LOG_DEBUG << vAcc.rotated(qRotationOffset) << "\n";
			static Median3Vec flt;

			if(fabs(vGyro.x) < 0.05) {
				vGyro.x *= 0.1;
			}
			if(fabs(vGyro.y) < 0.05) {
				vGyro.y *= 0.1;
			}
			if(fabs(vGyro.z) < 0.05) {
				vGyro.z *= 0.1;
			}

			flt.push(vGyro);
			flt.getValue(vGyro);

			//XPCC_LOG_DEBUG << vGyro << "\n";

			//rotated accelration vector
			Vector3f vAccRot = vAcc.rotated(qRotationOffset);

			madgwickAHRSUpdateIMU(vGyro.rotated(qRotationOffset),
					vAccRot, qRotation);



			static xpcc::Timeout<> offsetNull(2000);
			static uint8_t numSamples = 0;
			static Vector3f accelOffset;
			const uint8_t sampleCount = 100;

			if(offsetNull.isExpired()) {
				numSamples = sampleCount;
				offsetNull.restart(1000);
			}

			if(numSamples) {

				accelOffset += dynamicAcc;

				numSamples--;
				if(!numSamples) {
					accelOffset /= sampleCount;
				}
			}

			static VecFilter<filter::IIR<float, 10>> accF;

			accF.append(vAccRot);
			Vector3f v;
			accF.getValue(v);

			dynamicAcc = gravityCompensateAcc(v, qRotation) * 9.8;
			dynamicAcc -= accelOffset;

			velocity += dynamicAcc * (1.0 / 200);
			velocity *= 0.998;
		}

		//read sensors in round robin fashion
		switch(read) {
		case 0:
			if(gyro.read()) {
				read++;
			}
			break;
		case 1:
			if(accel.read()) {
				read++;
			}
			break;

		}

	}

private:
	uint16_t numSamples;
	xpcc::Timeout<> gyroCalib;
	xpcc::Vector3f gyroOffset;

	//accelerometer value at zero
	xpcc::Vector3f accZero;

	MMA8451Q<lpc17::I2cMaster2> accel;
	L3GD20<lpc17::I2cMaster2> gyro;

};



#endif /* SENSORPROCESSOR_HPP_ */
