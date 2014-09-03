/*
 * SensorProcessor.hpp
 *
 *  Created on: Apr 10, 2014
 *      Author: walmis
 */

#ifndef SENSORPROCESSOR_HPP_
#define SENSORPROCESSOR_HPP_

#include <xpcc/architecture.hpp>
#include <xpcc/math/filter.hpp>
#include <math.h>

#include "FXOS8700.hpp"

#include "MotionDriver/inv_mpu_dmp_motion_driver.h"
#include "MotionDriver/inv_mpu.h"

#include "madgwickAHRS.hpp"
#include "Ultrasonic.hpp"
#include "eedata.hpp"
#include "ms5611.hpp"

using namespace xpcc;

typedef Fp32f<30> Q30;

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

template <typename T>
class Average {
public:
        uint16_t count;
        T avg;

        T append(T val) {
        	avg = avg + ((val-avg) / ++count);
            return avg;
        }
        T getValue() {
                return avg;
        }

        void reset() {
                count = 0;
                avg = 0;
        }

};

template <typename T>
class HPF {
public:
	float coef;

	T sum;
	T lastSample;

	HPF(float coef) {
		this->coef = coef;
	}

	void append(T value) {
		sum = coef*(sum + value - lastSample);
		lastSample = value;
	}

	T getValue() {
		return sum;
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
	xpcc::Quaternion<float> qTrim;


	float ahrsHeading;
	float compassHeading;
	bool compassValid;

	float height;

	SensorProcessor() : qRotation(1.0, 0, 0, 0),
			qRotationOffset(1.0, 0, 0, 0),
			qTrim(1.0, 0, 0, 0)
	{
	}

	void handleInit() override {
		xpcc::delay_ms(50);

		if(eeprom.isValidToken()) {
			eeprom.eeRead(EEData::qTrim, qTrim);
			eeprom.eeRead(EEData::qRotationOffset, qRotationOffset);
		} else {
			eeprom.eeWrite(EEData::qTrim, qTrim);
			eeprom.eeWrite(EEData::qRotationOffset, qRotationOffset);
		}

		//mpu.initialize();
		//mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
		//mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

		//mpu.setI2CBypassEnabled(true);
		mpu_init_structures();

		mpu_init(0);

		long gyro[3], accel[3];
		mpu_run_self_test(gyro, accel);

		mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);// enable all of the sensors

		mpu_set_gyro_fsr(2000);

    	//mpu_set_gyro_bias_reg(gyro);

		mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);// get accel and gyro data in the FIFO also

		dmp_load_motion_driver_firmware();

		XPCC_LOG_DEBUG .printf("%d %d %d\n", gyro[0], gyro[1], gyro[2]);

		long gyroOffs[3] = {gyro[0]/2000, gyro[1]/2000, gyro[2]/2000};
		mpu_set_gyro_bias_reg(gyroOffs);
		//mpu_write_data(0x13, (uint8_t*)gyroOffs, 6);

		dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		        DMP_FEATURE_GYRO_CAL);

		dmp_set_fifo_rate(200);

		mpu_set_dmp_state(1);
		mpu_set_bypass(true);



		mag.initialize(0x1E);


//		if(!baro.initialize(0x77)) {
//			XPCC_LOG_DEBUG .printf("baro init failed\n");
//		}

		zero();
	}

	Vector3f getGravity(const Quaternion<float> &q) {
	    Vector3f v;
		v.x = 2 * (q.x*q.z - q.w*q.y);
	    v.y = 2 * (q.w*q.x + q.y*q.z);
	    v.z = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	    return v;
	}

	Vector3f getYawPitchRoll(const Quaternion<float> &q, const Vector3f &gravity) {
		Vector3f data;
		// yaw: (about Z axis)
		data[0] = atan2f(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1);

		//pitch
		data.y = atanf(gravity.y / gravity.z);
		if (gravity.z < 0.0)
		{
			if(gravity.y > 0.0) {
				data.y += M_PI;
			} else {
				data.y -= M_PI;
			}
		}

		//roll
		data.z = -atanf(gravity.x / gravity.z);
		if (gravity.z < 0.0)
		{
			if(gravity.x > 0.0) {
				data.z -= M_PI;
			} else {
				data.z += M_PI;
			}
		}

		return data;
	}


	xpcc::Vector3f getEuler() {
		return qRotation.toEuler() * 180.0/M_PI;
	}

	void applyTrim(Quaternion<float> &q) {
		qRotationOffset = qRotationOffset * qTrim.conjugated();
		qRotationOffset =  qRotationOffset * q;
		qRotationOffset.normalize();
		qTrim = q;

		eeprom.eeWrite(EEData::qTrim, q);
	}

	void setRotationOffset(Quaternion<float> &q) {
		qRotationOffset = q;
		qTrim = Quaternion<float>(1.0, 0,0,0);

		eeprom.eeWrite(EEData::qRotationOffset, q);
	}

	void zero() {
		gyroCalib.restart(500);
		ledGreen::set();

		accZero = 0;
		gyroOffset = 0;

		tCalibrated = false;
		numSamples = 0;
	}

	void zeroRotation() {
		qRotationOffset = Quaternion<float>(1.0,0,0,0);
		zero();
	}

	bool zeroingComplete() {
		return !gyroCalib.isActive();
	}

	Vector3f getYPR(Quaternion<float> &q, const Vector3f &gravity) {
		  Vector3f ypr;

		  ypr[0] = atan2f(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
		  ypr[1] = atanf(gravity[0] / sqrtf(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
		  ypr[2] = atanf(gravity[1] / sqrtf(gravity[0]*gravity[0] + gravity[2]*gravity[2]));

		  return ypr;
	}

	float getHeading(Quaternion<float> &q) {
		return -atan2f(2*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
	}

	Vector2f getHeadingVector(Quaternion<float> &q) {
		Vector2f v(2*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
		v.normalize();
		return v;
	}

	float difangrad(float x, float y)
	{
		float arg;

		arg = fmodf(y-x, 2*M_PI);
		if (arg < 0 )  arg  = arg + 2*M_PI;
		if (arg > M_PI) arg  = arg - 2*M_PI;

		return arg;
	}

	void handleTick() {
		static PeriodicTimer<> tUsnd(20);
		static PeriodicTimer<> timerRead(5);

		static Timeout<> mpuTimer(4);
		static PeriodicTimer<> magTimer(20);

		static Timestamp tRead = lpc17::RitClock::now();

		static uint8_t read = 0;

		if(mpuTimer.isExpired()) {
			int16_t intStatus;

		    mpu_get_int_status(&intStatus);                       // get the current MPU state
		    if(intStatus & MPU_INT_STATUS_FIFO_OVERFLOW) {
		    	XPCC_LOG_DEBUG << "FIFO overflow\n";
		    	mpu_reset_fifo();
		    } else
		    if ((intStatus & MPU_INT_STATUS_DMP_0)) {         // return false if definitely not ready yet
		    	uint32_t timestamp;
		    	int16_t gyro[3], accel[3];
		    	int32_t quat[4];
		    	int16_t sensors;
		    	uint8_t more;

		    	dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);

		    	Quaternion<Q30> q;
		    	q.w.rawVal = quat[0];
		    	q.x.rawVal = quat[1];
		    	q.y.rawVal = quat[2];
		    	q.z.rawVal = quat[3];

		    	qRotation = q;

		    	qRotation = {quat[0], quat[1], quat[2], quat[3]};
		    	qRotation.normalize();

		    	vGyro.x = gyro[0]/16.4;
		    	vGyro.y = gyro[1]/16.4;
		    	vGyro.z = gyro[2]/16.4;

		    	vAcc.x = accel[0]/16384.0;
		    	vAcc.y = accel[1]/16384.0;
		    	vAcc.z = accel[2]/16384.0;

		    	Fp32f<16> a = 25.6;
		    	Fp32f<23> b = 10.0;

		    	a = b;

		    	//XPCC_LOG_DEBUG << timestamp << qRotation << vAcc << vGyro<< endl;


		    	//static float headingCompensation;
		    	//Quaternion<float> tmp(Vector3f(0, 0, 1), headingCompensation);

		    	//qRotation = tmp * qRotation;

		    	//ahrsHeading = getHeading(qRotation);

		    	//float e = difangrad(compassHeading, ahrsHeading);

		    	//headingCompensation += e*0.0005;

		    	if(!more)
		    		mpuTimer.restart(4);
		    }

		}

		if(magTimer.isExpired()) {
			int16_t mx, my, mz;

			mag.readMagData(&mx, &my, &mz);

			vMag.x = -mx * 0.1;
			vMag.y = -my * 0.1;
			vMag.z = mz * 0.1;


			//compass tilt compensation
			vMag.rotate(qRotation);
			//vMag.normalize();

			///

			//XPCC_LOG_DEBUG << vMag << endl;


			float angle = 45 * M_PI/180.0;
			Vector3f axis(0.0,0.0,1.0);

			axis.rotate(qRotation);
			XPCC_LOG_DEBUG << axis <<endl;

			Quaternion<float> qt(axis, angle);

			XPCC_LOG_DEBUG << qt <<endl;
//qRotation = qRotation * qt.conjugated();
			qt = qRotation * qt.conjugated();

			XPCC_LOG_DEBUG << '$';
			XPCC_LOG_DEBUG .write((uint8_t*)&qRotation, sizeof(float)*4);
			XPCC_LOG_DEBUG .write((uint8_t*)&qRotation, sizeof(float)*4);
			XPCC_LOG_DEBUG .write((uint8_t*)&vMag, sizeof(float)*3);


			//XPCC_LOG_DEBUG << "hdg " << ahrsHeading*180.0/M_PI << " mag " << compassHeading*180.0/M_PI + 5.5<< endl;
		}


		if(0 && tUsnd.isExpired()) {
			//static xpcc::filter::LPF<uint32_t> filter(4);

			//uint32_t t = alt.getTime();
			//XPCC_LOG_DEBUG .printf("h %d\n", t);

				//static float f;

//				filter.append(t);
//
//				float usnd = (t / (float)SystemCoreClock) * 334.0/2.0;
//
//				float baro_alt = baro.getAltitude();
//
//				static float baro_offset = 0;
//				if(usnd < 0.1) {
//					baro_offset = baro_alt - usnd;
//				}
//				if(usnd > 1.0) {
//					usnd = baro_alt - baro_offset;
//				}

//				const float k_vz = -0.0001;
//				const float k_h_est = -0.008;

				const float dt = 0.02;

//				static float vz_est;
//				static float h_est;

//				vz_est = vz_est + dynamicAcc.z * dt;
//				vz_est *= 0.998;
//				h_est = h_est + vz_est * dt;
//
//				vz_est = vz_est + k_vz * (h_est - baro_alt);
//				h_est = h_est + k_h_est * (h_est - baro_alt);

				//height = baro.pressure;

				//XPCC_LOG_DEBUG .printf("%.4f %.4f\n", baro_alt, usnd);

//				float Kv = 0.4;
//				float Kusnd = 0.1;
//
//				if(!t) {
//					Kusnd = 0;
//				}
//
//
//				//f = Kv * (f + velocity.z * dt) + 0.5 * (baro_alt - baro_offset) + Kusnd * (usnd);
//
//				height = f;
//
//				onHeightUpdated(0.020);
//
//
//			alt.ping();
		}

		if(0 && timerRead.isExpired()) {
			read = 0; //reset read order

			float dt = (lpc17::RitClock::now() - tRead).getTime() / (float)SystemCoreClock;
			tRead = lpc17::RitClock::now();

			int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

			//mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);



			vMag.x = -mx * 0.1;
			vMag.y = -my * 0.1;
			vMag.z = mz * 0.1;
			//XPCC_LOG_DEBUG .printf("%d %d %d %d %d %d\n", ax, ay, az, gx, gy, gz);


			vGyro.x = -gx / 32.8;
			vGyro.y = -gy / 32.8;
			vGyro.z = gz / 32.8;

			vAcc.x = -ax / 8192.0;
			vAcc.y = -ay / 8192.0;
			vAcc.z = az / 8192.0;


			vGyro *= M_PI/180.0; // convert reading to rad/s

			//static float gMagnitude;


			if(gyroCalib.isActive() && gyroCalib.isExpired()) {
				gyroCalib.stop();

				accZero /= numSamples;
				gyroOffset /= numSamples;

				//gMagnitude = accZero.getLength();

				XPCC_LOG_DEBUG .printf("calibration complete\n");
				XPCC_LOG_DEBUG << "Gyro offset:" << gyroOffset << "\n";
				XPCC_LOG_DEBUG << "Acc zero value: " << accZero << "\n";

				if(qRotationOffset == Quaternion<float>(1.0, 0, 0, 0)) {
					auto g = Vector3f(0,0,1);
					auto z = accZero;
					auto cross = z.cross(g);
					qRotationOffset = Quaternion<float>(
							sqrtf(z.getLengthSquared() * g.getLengthSquared())
									+ z * g, cross.x, cross.y, cross.z);
					qRotationOffset.normalize();

					eeprom.eeWrite(EEData::qRotationOffset, qRotationOffset);

					XPCC_LOG_DEBUG << "Q - " << qRotationOffset << "\n";

				}

				accZero.rotate(qRotationOffset);
				//accZero = Vector3f(0, 0, 1) - accZero.rotate(qRotationOffset);

				tCalibrated = xpcc::Clock::now().getTime();

				onSensorsCalibrated();

				//reset rotation
				qRotation = Quaternion<float>(1.0,0,0,0);
			}


//			if(gyroCalib.isActive()) {
//				accZero += vAcc;
//				gyroOffset += vGyro;
//				numSamples++;
//			} else {
//				vGyro -= gyroOffset;
//
//				if(tCalibrated) {
//					static Vector3f sum;
//
//					sum += vGyro;
//					if(xpcc::Clock::now().getTime() - tCalibrated > 500) {
//						ledGreen::reset();
//
//						for(int i = 0; i < 3; i++) {
//							if(fabs(sum[i]) > 0.03) {
//								zero();
//								break;
//							}
//						}
//
//						sum = 0;
//						tCalibrated = 0;
//					}
//				}


	//			static Average<Vector3f> avg;
	//			if(!calibrated) {
	//				avg.append(vGyro);
	//
	//
	//
	//
	//			}

	//			if(fabs(vGyro.x) < 0.005) {
	//				vGyro.x = 0;
	//			}
	//			if(fabs(vGyro.y) < 0.005) {
	//				vGyro.y = 0;
	//			}
	//			if(fabs(vGyro.z) < 0.005) {
	//				vGyro.z = 0;
	//			}

				//XPCC_LOG_DEBUG << vAcc.rotated(qRotationOffset) << "\n";

				//flt.push(vGyro);
				//flt.getValue(vGyro);

				//XPCC_LOG_DEBUG << vGyro << "\n";

				//rotated accelration vector
	//			Vector3f vAccRot, vGyroRot;
	//			if(true) {
	//				vAccRot = vAcc.rotated(qRotationOffset);
	//				vGyroRot = vGyro.rotated(qRotationOffset);
	//			} else {
	//				vAccRot = vAcc;
	//				vGyroRot = vGyro;
	//			}

				//vAcc.rotate(qRotationOffset);
				//vGyro.rotate(qRotationOffset);

				//madgwickAHRSUpdateIMU(vGyro, vAcc, qRotation, dt);

				//static filter::LPF<Vector3f> accF(10);

				//static VecFilter<filter::IIR<float, 30>> gF;

//				Vector3f g = getGravity(qRotation);
//				gF.append(g);
//				gF.getValue(g);

				//accF.append(vAcc);
				//vAcc = accF.getValue();

				//dynamicAcc = vAcc.rotated(qRotation);
				//dynamicAcc.z -= accZero.z;
				//dynamicAcc *= 9.8;

				//dynamicAcc = (vAcc - g*gMagnitude) * 10;
	//			if(fabs(dynamicAcc.x) < 0.1) {
	//				dynamicAcc.x = 0;
	//			}
	//			if(fabs(dynamicAcc.y) < 0.1) {
	//				dynamicAcc.y = 0;
	//			}
	//			if(fabs(dynamicAcc.z) < 0.1) {
	//				dynamicAcc.z = 0;
	//			}
				//velocity += dynamicAcc * dt;
				//velocity *= 0.998;

				//onSensorsUpdated(dt);
			//}
		}


	}

	virtual void onHeightUpdated(float dt) {}
	virtual void onSensorsUpdated(float dt) {}
	virtual void onSensorsCalibrated() {}

//private:
	uint32_t tCalibrated; //time when calibration is complete

	uint16_t numSamples; //number of calibration samples collected
	xpcc::Timeout<> gyroCalib; //gyro calibration timeout

	Median3Vec accMedian;

	//accelerometer value at zero
	xpcc::Vector3f accZero;
	xpcc::Vector3f gyroOffset;


	//MPU6050 mpu;
	FXOS8700 mag;

	Ultrasonic<usnd0_trig, usnd0_echo, lpc17::RitClock> alt;

};



#endif /* SENSORPROCESSOR_HPP_ */
