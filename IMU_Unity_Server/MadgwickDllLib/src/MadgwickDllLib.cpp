// MadgwickDllLib.cpp : Defines the exported functions for the DLL application.
//
#include "IUnityInterface.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "MathAlignment.h"
#include "MathEigen.h"
#include "MathUtility.h"

Eigen::Quaternionf currentRotation;


//Eigen::Quaternionf currentRotation = Eigen::Quaternionf::Identity();

// -- OrientationFilterMadgwickARG --
// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
extern "C" UNITY_INTERFACE_EXPORT Eigen::Quaternionf  UNITY_INTERFACE_API
 MadgwickFilter(float dt, const Eigen::Vector3f * gyro, const Eigen::Vector3f * acc, const Eigen::Vector3f * mag, const Eigen::Vector3f &gyro_variance,  const Eigen::Quaternionf &currentRotation)
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
//	currentRotation = SEq_new;

	return SEq_new; //currentRotation
}
/*
extern "C" UNITY_INTERFACE_EXPORT Eigen::Quaternionf  UNITY_INTERFACE_API
MadgwickUpdate(float dt, const Eigen::Vector3f  gyro, const Eigen::Vector3f  acc, const Eigen::Vector3f  mag, Eigen::Vector3f gyro_variance, Eigen::Quaternionf currentRotation)
{
	currentRotation = MadgwickFilter(dt, &gyro, &acc, &mag, gyro_variance,currentRotation);
	return currentRotation;
}
*/
extern "C" UNITY_INTERFACE_EXPORT bool UNITY_INTERFACE_API
IsMadgwickDllLibLoaded()
{
	return true;
}