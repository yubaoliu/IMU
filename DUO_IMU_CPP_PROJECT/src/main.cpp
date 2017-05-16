#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include "UdpBaseBehaviour.h"
#include "Sample.h"
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "MathAlignment.h">
#include "MathEigen.h"
#include "MathUtility.h"
#include <cmath>
#include <thread>

#define WIDTH	640 
#define HEIGHT 480
#define FPS	30

Eigen::Quaternionf currentRotation= Eigen::Quaternionf::Identity();
#define DUO_IMU_CALIBRATION
#define M_PI 3.1415926
void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);
Eigen::Quaternionf Update(float dt, const Eigen::Vector3f * gyro, const Eigen::Vector3f * acc, const Eigen::Vector3f * mag, Eigen::Vector3f gyro_variance);
//Eigen::Vector3f removeGravity(Eigen::Vector3f accel, Eigen::Quaternionf q);
void parameterAdjustment();
double deg2rad(double deg);
#ifdef DUO_IMU_CALIBRATION
std::fstream fs_IMU_original; //for Gyo
std::fstream fs_IMU_kalmanfilter; //for Gyo
#endif
std::fstream fs_IMU_final; //for Gyo
std::fstream fs_IMU_accelerator; //for accelerator

Udp_base_behaviour *udpEnd = new Udp_base_behaviour();
char sendBuf[100] = {"Hello world"};
char recvBuf[1024] = { '\0' };
bool startRecvThread = false;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

//kalman filter parameter init
const int stateNum = 3;
const int measureNum = 3;
cv::KalmanFilter KF(stateNum, measureNum, 0);	//state: Gyo_x, Gyo_y, Gyo_z
cv::Mat state(stateNum, 1, CV_32FC1);
cv::Mat processNoise(stateNum, 1, CV_32F);
cv::Mat measureMent = cv::Mat::zeros(measureNum, 1, CV_32F);


//Eigen::Vector3f _gyro_offset = Eigen::Vector3f(-1.9175f, 1.9869f, 0.1148f);//work well
Eigen::Vector3f _gyro_offset = Eigen::Vector3f(-1.8966f, 1.9159f, 0.1865f);
//Eigen::Vector3f _gyro_offset = Eigen::Vector3f(0.0f,0.0f,0.0f);

//Eigen::Vector3f _gyro_varience=Eigen::Vector3f(deg2rad(-1.9175f), deg2rad(1.9869f), deg2rad(0.1148f));
//Eigen::Vector3f _gyro_varience = Eigen::Vector3f(0.03f, 0.03f, 0.03f);
Eigen::Vector3f _gyro_varience = Eigen::Vector3f(0.0351f, 0.0149f, 0.0000001f);

struct IMU_STRUCT {
	Eigen::Vector3f linear_acceleration;
	Eigen::Vector3f angular_velocity;
	Eigen::Vector3f magn = Eigen::Vector3f::Zero();
//	float orientation_covariance;
//	float angular_velocity_covariance;
//	float linear_acceleration_covariance;
}imu_msg;

int main(int argc, char* argv[])
{	
	/*Log file init*/
	fs_IMU_kalmanfilter.open("../matlab/kalmanfilterIMU.txt", std::fstream::out);
	fs_IMU_original.open("../matlab/originalIMU.txt", std::fstream::out);
	fs_IMU_final.open("../matlab/finalIMU.txt", std::fstream::out);
	fs_IMU_accelerator.open("../matlab/accelerator.txt", std::fstream::out);

#ifdef KALMAN_FILTER
	//kalman fileter
	KF.transitionMatrix = cv::Mat::eye(3, 3, CV_32F);
	cv::setIdentity(KF.measurementMatrix);
	cv::setIdentity(KF.processNoiseCov,cv::Scalar::all(1e-5));
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(KF.errorCovPost, cv::Scalar(1));
#endif
	//UDP communication
	udpEnd->Winsock_init();
	udpEnd->InitRemoteSocket_Addr_Port("127.0.0.1", 8888);
	udpEnd->InitLocalSocket_LocalAddr_Port("127.0.0.1", 8888);
	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());
	std::thread th(parameterAdjustment);

	if (!OpenDUOCamera(WIDTH, HEIGHT, FPS))
	{
		printf("Could not open DUO camera\n");
		return 0;
	}

	cv::namedWindow("Left");
	cv::namedWindow("Right");

	SetExposure(80);
	SetLed(50);
	// Sets sampling rate
	SetIMURate(500); //very important, the deltaTime parameters filter method need to be calculated according to it

	// Create image headers for left & right frames     
	cv::Mat left(cv::Size(WIDTH, HEIGHT), CV_8UC1);
	cv::Mat right(cv::Size(WIDTH, HEIGHT), CV_8UC1);

	while ((cv::waitKey(1) & 0xff) != 27)
	{
		PDUOFrame pFrameData = GetDUOFrame();
		if (pFrameData == NULL) continue;
		if (pFrameData->IMUPresent)
		{
			for (int i = 0; i < pFrameData->IMUSamples; i++)
			{
				fs_IMU_original << pFrameData->IMUData[i].gyroData[0] << "," << pFrameData->IMUData[i].gyroData[1] << "," << pFrameData->IMUData[i].gyroData[2] << std::endl;
				imu_msg.linear_acceleration[0] = pFrameData->IMUData[i].accelData[0] * 9.80151f;
				imu_msg.linear_acceleration[1] = pFrameData->IMUData[i].accelData[1] * 9.80151f;
				imu_msg.linear_acceleration[2] = pFrameData->IMUData[i].accelData[2] * 9.80151f;

				imu_msg.angular_velocity[0] = deg2rad(pFrameData->IMUData[i].gyroData[0] - _gyro_offset.x());
				imu_msg.angular_velocity[1] = deg2rad(pFrameData->IMUData[i].gyroData[1] - _gyro_offset.y());
				imu_msg.angular_velocity[2] = deg2rad(pFrameData->IMUData[i].gyroData[2] - _gyro_offset.z());

				currentRotation = Update((2.0f / 1000.0f), &imu_msg.angular_velocity, &imu_msg.linear_acceleration, &imu_msg.magn, _gyro_varience);

				float q0 = currentRotation.x();
				float q1 = currentRotation.y();
				float q2 = currentRotation.z();
				float q3 = currentRotation.w();
				char end = 'E';
				//send to UNITY client
				memset(sendBuf, 0, sizeof(sendBuf));
				memcpy(sendBuf, (char *)&q0, sizeof(float));
				memcpy(sendBuf + sizeof(float), (char *)&q1, sizeof(float));
				memcpy(sendBuf + sizeof(float) * 2, (char *)&q2, sizeof(float));
				memcpy(sendBuf + sizeof(float) * 3, (char *)&q3, sizeof(float));
				memcpy(sendBuf + sizeof(float) * 4, &end, sizeof(char));
				udpEnd->SendUdpPacket(sendBuf, sizeof(sendBuf));

				roll = currentRotation.toRotationMatrix().eulerAngles(0, 1, 2).x();
				pitch = currentRotation.toRotationMatrix().eulerAngles(0, 1, 2).y();
				yaw = currentRotation.toRotationMatrix().eulerAngles(0, 1, 2).z();
				fs_IMU_final << roll << "," << pitch << "," << yaw << std::endl;

				startRecvThread = true;
			}
		}

		left.data = (uchar*)pFrameData->leftData;
		right.data = (uchar*)pFrameData->rightData;

		cv::imshow("Left", left);
		cv::imshow("Right", right);

		cv::imwrite("D:\\home\\GIT_PROJECT\\IMU\\IMU_Unity_Server\\Assets\\Asset\\Resources\\scene.jpg", left);
		
	}

	//clean
	fs_IMU_kalmanfilter.close();
	fs_IMU_original.close();
	fs_IMU_final.close();
	fs_IMU_accelerator.close();
	
	th.join();
	th.detach();

	CloseDUOCamera();
	return 0;
}
#if 0
void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
		printf("DUO Frame Timestamp: %10.1f ms\n", pFrameData->timeStamp / 10.0f);
		if (pFrameData->IMUPresent)
		{
			for (int i = 0; i < pFrameData->IMUSamples; i++)
			{
				fs_IMU_original << pFrameData->IMUData[i].gyroData[0] << "," << pFrameData->IMUData[i].gyroData[1] << "," << pFrameData->IMUData[i].gyroData[2] << std::endl;
				imu_msg.linear_acceleration[0] = pFrameData->IMUData[i].accelData[0] * 9.80151f;
				imu_msg.linear_acceleration[1] = pFrameData->IMUData[i].accelData[1] * 9.80151f;
				imu_msg.linear_acceleration[2] = pFrameData->IMUData[i].accelData[2] * 9.80151f;

				imu_msg.angular_velocity[0] = deg2rad(pFrameData->IMUData[i].gyroData[0] - _gyro_offset.x());
				imu_msg.angular_velocity[1] = deg2rad(pFrameData->IMUData[i].gyroData[1] - _gyro_offset.y());
				imu_msg.angular_velocity[2] = deg2rad(pFrameData->IMUData[i].gyroData[2] - _gyro_offset.z());

				//imu_msg.linear_acceleration = removeGravity(imu_msg.linear_acceleration, currentRotation);

				currentRotation = Update((2.0f/1000.0f), &imu_msg.angular_velocity, &imu_msg.linear_acceleration, &imu_msg.magn, _gyro_varience);
	
				float q0 = currentRotation.x();
				float q1 = currentRotation.y();
				float q2 = currentRotation.z();
				float q3 = currentRotation.w();
				char end = 'E';
				//send to UNITY client
				memset(sendBuf, 0, sizeof(sendBuf));
				memcpy(sendBuf, (char *)&q0, sizeof(float));
				memcpy(sendBuf + sizeof(float), (char *)&q1, sizeof(float));
				memcpy(sendBuf + sizeof(float) * 2, (char *)&q2, sizeof(float));
				memcpy(sendBuf + sizeof(float) * 3, (char *)&q3, sizeof(float));
				memcpy(sendBuf + sizeof(float) * 4, &end, sizeof(char));
				udpEnd->SendUdpPacket(sendBuf, sizeof(sendBuf));

				roll = currentRotation.toRotationMatrix().eulerAngles(0,1,2).x();
				pitch = currentRotation.toRotationMatrix().eulerAngles(0,1,2).y();
				yaw = currentRotation.toRotationMatrix().eulerAngles(0,1,2).z();
				fs_IMU_final << roll << "," << pitch << "," << yaw << std::endl;

				startRecvThread = true;
#endif

#if 0

						// Angular velocity should be in rad/sec
					printf("  Accelerometer: [%8.5f, %8.5f, %8.5f]\n", pFrameData->IMUData[i].accelData[0],
						pFrameData->IMUData[i].accelData[1],
						pFrameData->IMUData[i].accelData[2]);
					fs_IMU_accelerator << pFrameData->IMUData[i].accelData[0] << "," << pFrameData->IMUData[i].accelData[1] << "," << pFrameData->IMUData[i].accelData[2] << std::endl;
					printf("  Gyro:          [%8.5f, %8.5f, %8.5f]\n", pFrameData->IMUData[i].gyroData[0],
						pFrameData->IMUData[i].gyroData[1],
						pFrameData->IMUData[i].gyroData[2]);

					//prediction
					cv::Mat prediction = KF.predict();
					float predictGyo_x = prediction.at<float>(0, 0);
					float predictGyo_y = prediction.at<float>(1, 0);
					float predictGyo_z = prediction.at<float>(2, 0);

					//corect
					measureMent.at<float>(0, 0) = imu_msg.angular_velocity[0];
					measureMent.at<float>(1, 0) = imu_msg.angular_velocity[1];
					measureMent.at<float>(2, 0) = imu_msg.angular_velocity[2];

					KF.correct(measureMent);
					state = KF.transitionMatrix*state;
					fs_IMU_kalmanfilter << predictGyo_x << "," << predictGyo_y << "," << predictGyo_z << std::endl;

				}
					/*Bias of zero*/
					roll = (pFrameData->IMUData[i].gyroData[0] - mean[0]);
					pitch = (pFrameData->IMUData[i].gyroData[0] - mean[1]);
					yaw = (pFrameData->IMUData[i].gyroData[0] - mean[2]);

					

					//send to UNITY client
					memset(buf, 0, sizeof(buf));
					memcpy(buf, (char *)&roll, sizeof(float));
					memcpy(buf + sizeof(float), (char *)&pitch, sizeof(float));
					memcpy(buf + sizeof(float) * 2, (char *)&yaw, sizeof(float));
					udpEnd->SendUdpPacket(buf, sizeof(buf));
				

				if (_num_samples < 101) {
					_num_samples++;
				}
#endif
			//}
		//}//for (int i = 0; i < pFrameData->IMUSamples; i++)

		
//		printf("------------------------------------------------------\n");
	
//}


// -- OrientationFilterMadgwickARG --
// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
Eigen::Quaternionf Update(float dt, const Eigen::Vector3f * gyro, const Eigen::Vector3f * acc, const Eigen::Vector3f * mag, Eigen::Vector3f gyro_variance)
{
	const Eigen::Vector3f &current_omega = Eigen::Vector3f(gyro->x(), gyro->y(), gyro->z());

	Eigen::Vector3f current_g = Eigen::Vector3f(acc->x(), acc->y(), acc->z());
	eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

	// Current orientation from earth frame to sensor frame
	const Eigen::Quaternionf SEq = currentRotation;
	Eigen::Quaternionf SEq_new = SEq;

	// Compute the quaternion derivative measured by gyroscopes
	// Eqn 12) q_dot = 0.5*q*omega
	Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
	Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) *omega;

	if (!current_g.isApprox(Eigen::Vector3f::Zero(), k_normal_epsilon))
	{
		// Get the direction of the gravitational fields in the identity pose
		Eigen::Vector3f k_identity_g_direction = Eigen::Vector3f(0, -0.9, 0); // m_constants.gravity_calibration_direction;

																			  // Eqn 15) Applied to the gravity vector
																			  // Fill in the 3x1 objective function matrix f(SEq, Sa) =|f_g|
		Eigen::Matrix<float, 3, 1> f_g;
		eigen_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

		// Eqn 21) Applied to the gravity vector
		// Fill in the 4x3 objective function Jacobian matrix: J_gb(SEq)= [J_g]
		Eigen::Matrix<float, 4, 3> J_g;
		eigen_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

		// Eqn 34) gradient_F= J_g(SEq)*f(SEq, Sa)
		// Compute the gradient of the objective function
		Eigen::Matrix<float, 4, 1> gradient_f = J_g * f_g;
		Eigen::Quaternionf SEqHatDot =
			Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

		// normalize the gradient
		eigen_quaternion_normalize_with_default(SEqHatDot, *k_eigen_quaternion_zero);

		// Compute the estimated quaternion rate of change
		// Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot

		//float guess = 0.03; //rad/s -- TODO: this has to be  measured in a calibration routine
		//Eigen::Vector3f gyro_variance = Eigen::Vector3f(guess, guess, guess);

		const float beta = sqrtf(3.0f / 4.0f) * fmaxf(fmaxf(gyro_variance.x(), gyro_variance.y()), gyro_variance.z());
		Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*beta);

		// Compute then integrate the estimated quaternion rate
		// Eqn 42) SEq_new = SEq + SEqDot_est*delta_t
		SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*dt);
	}
	else
	{
		SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_omega.coeffs()*dt);
	}

	// Make sure the net quaternion is a pure rotation quaternion
	SEq_new.normalize();

	// Saves the new orientation value
	currentRotation = SEq_new;

	return currentRotation;
}
double deg2rad(double deg) {
	return deg * M_PI / 180.0;
}

void parameterAdjustment()
{
	while (1)
	{
		if (startRecvThread)
		{
			char endLable;
			udpEnd->ReceivePacket(recvBuf, sizeof(recvBuf));
			memcpy(&(_gyro_offset.x()), recvBuf, sizeof(float));
			memcpy(&(_gyro_offset.y()), recvBuf + 1 * sizeof(float), sizeof(float));
			memcpy(&(_gyro_offset.z()), recvBuf + 2 * sizeof(float), sizeof(float));
			memcpy(&(_gyro_varience.x()), recvBuf + 3 * sizeof(float), sizeof(float));
			memcpy(&(_gyro_varience.y()), recvBuf + 4 * sizeof(float), sizeof(float));
			memcpy(&(_gyro_varience.z()), recvBuf + 5 * sizeof(float), sizeof(float));
			memcpy(&endLable, recvBuf + 6 * sizeof(float), sizeof(char));
			//assert(endLable == 'E');
			memset(recvBuf, 0, sizeof(recvBuf));
		}
	}
}