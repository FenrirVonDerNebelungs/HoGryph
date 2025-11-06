#include "RotationTrans.h"
RotationTrans::RotationTrans() {
	;
}
RotationTrans::~RotationTrans() {
	;
}
Quaternion RotationTrans::Quaternion_From_Euler(
	const float& phi,
	const float& theta,
	const float& psi
) const {
	Quaternion_Rotation quat_rotor;
	Vector X(1.0f, 0.0f, 0.f);
	Vector Y(0.f, 1.f, 0.f);
	Vector Z(0.f, 0.f, 1.f);
	/* three rotations 1st rotation axis is z */
	Vector R_phi_axis=Z;
	Axis_Angle Rot_1(R_phi_axis, phi);
	Vector R_theta_axis = quat_rotor.Rotate(Rot_1, X);
	Axis_Angle Rot_2(R_theta_axis, theta);
	Vector R_psi_axis = quat_rotor.Rotate(Rot_2, Z);
	Axis_Angle Rot_3(R_psi_axis, psi);
	/* find q_psi*q_theta*q_phi */
	Quaternion q_phi = quat_rotor.GetRotationQuaternion(Rot_1);
	Quaternion q_theta = quat_rotor.GetRotationQuaternion(Rot_2);
	Quaternion q_psi = quat_rotor.GetRotationQuaternion(Rot_3);
	Quaternion q_psi_theta_phi = quat_rotor.CombineRotation(q_phi, q_theta, q_psi);
	return q_psi_theta_phi;
}
void RotationTrans::Euler_From_Quaternion(
	const Quaternion& q,
	float& phi,
	float& theta,
	float& psi
) const {
	Quaternion_Rotation quat_rotor;
	/*quaternion to matrix*/
	Matrix Rot_M = quat_rotor.GetRotationMatrix(q);
	Euler_Rotation euler_rotor;
	euler_rotor.GetRotationAngles(Rot_M, phi, theta, psi);
}
Axis_Angle RotationTrans::CombineRotation_UsingQuaternions(
	const Axis_Angle& u_rot_1,
	const Axis_Angle& u_rot_2
) const {
	Quaternion_Rotation quat_rotor;
	Quaternion q1 = quat_rotor.GetRotationQuaternion(u_rot_1);
	Quaternion q2 = quat_rotor.GetRotationQuaternion(u_rot_2);
	Quaternion q = quat_rotor.CombineRotation(q1, q2);
	Axis_Angle combined_rot = quat_rotor.GetRotationAxis(q);
	return combined_rot;
}
