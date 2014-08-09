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
#include "MPU6050.hpp"
#include "madgwickAHRS.hpp"
#include "Ultrasonic.hpp"
#include "eedata.hpp"
#include "ms5611.hpp"

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

		mpu.initialize();
		mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
		mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

		mpu.setI2CBypassEnabled(true);

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

	void handleTick() {
		static PeriodicTimer<> tUsnd(20);
		static PeriodicTimer<> timerRead(5);
		static Timestamp tRead = lpc17::RitClock::now();

		static uint8_t read = 0;

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

		if(timerRead.isExpired() && mpu.isMotionDataReady() && mag.isMagReady()) {
			read = 0; //reset read order

			float dt = (lpc17::RitClock::now() - tRead).getTime() / (float)SystemCoreClock;
			tRead = lpc17::RitClock::now();

			int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

			mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			mag.getMagData(&mx, &my, &mz);

			vMag.x = mx * 0.1;
			vMag.y = my * 0.1;
			vMag.z = mz * 0.1;
			//XPCC_LOG_DEBUG .printf("%d %d %d %d %d %d\n", ax, ay, az, gx, gy, gz);

			//XPCC_LOG_DEBUG .printf("%d %d %d\n", mx, my, mz);
			//accel.getXYZ(vAcc);
			//gyro.getXYZ(vGyro);

			vGyro.x = -gx / 32.8;
			vGyro.y = -gy / 32.8;
			vGyro.z = gz / 32.8;

			vAcc.x = -ax / 8192.0;
			vAcc.y = -ay / 8192.0;
			vAcc.z = az / 8192.0;

			//accMedian.push(vAcc);
			//accMedian.getValue(vAcc);

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


			if(gyroCalib.isActive()) {
				accZero += vAcc;
				gyroOffset += vGyro;
				numSamples++;
			} else {
				vGyro -= gyroOffset;

				if(tCalibrated) {
					static Vector3f sum;

					sum += vGyro;
					if(xpcc::Clock::now().getTime() - tCalibrated > 500) {
						ledGreen::reset();

						for(int i = 0; i < 3; i++) {
							if(fabs(sum[i]) > 0.03) {
								zero();
								break;
							}
						}

						sum = 0;
						tCalibrated = 0;
					}
				}


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

				vAcc.rotate(qRotationOffset);
				vGyro.rotate(qRotationOffset);

				madgwickAHRSUpdateIMU(vGyro, vAcc, qRotation, dt);

				static filter::LPF<Vector3f> accF(10);

				//static VecFilter<filter::IIR<float, 30>> gF;

//				Vector3f g = getGravity(qRotation);
//				gF.append(g);
//				gF.getValue(g);

				accF.append(vAcc);
				vAcc = accF.getValue();

				dynamicAcc = vAcc.rotated(qRotation);
				dynamicAcc.z -= accZero.z;
				dynamicAcc *= 9.8;

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
				velocity += dynamicAcc * dt;
				velocity *= 0.998;

				onSensorsUpdated(dt);
			}
		}

		//read sensors in round robin fashion
		switch(read) {
		case 0:
			if(mpu.startReadMotion6()) {
				read++;
			}
			break;
		case 1:
			if(mag.startReadMag()) {
				read++;
			}
			break;

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


	MPU6050 mpu;
	FXOS8700 mag;

	Ultrasonic<usnd0_trig, usnd0_echo, lpc17::RitClock> alt;

};



#endif /* SENSORPROCESSOR_HPP_ */
