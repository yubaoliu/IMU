#ifndef _IMU_H_
#define _INU_H_
#include <Eigen/Core>
#include <Eigen/Geometry>


struct TrackerInput
{
	Eigen::Vector3f	gyro;
	Eigen::Vector3f	acce;
	Eigen::Vector3f	magn;
	uint32_t timeStamp;
};




#endif