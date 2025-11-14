#include "Euler.h"
Euler_Rotation::Euler_Rotation() {
	;
}
Euler_Rotation::~Euler_Rotation() {
	;
}
Matrix Euler_Rotation::GetRx(const float& ang) const {
	Vector Row0(1.f, 0.f, 0.f);
	Vector Row1(0.f, cosf(ang), sinf(ang));
	Vector Row2(0.f, -sinf(ang), cosf(ang));
	Matrix R;
	R.SetRow(0, Row0);
	R.SetRow(1, Row1);
	R.SetRow(2, Row2);
	return R;
}
Matrix Euler_Rotation::GetRy(const float& ang) const {
	Vector Row0(cosf(ang), 0.f, -sinf(ang));
	Vector Row1(0.f, 1.f, 0.f);
	Vector Row2(sinf(ang), 0.f, cosf(ang));
	Matrix R;
	R.SetRow(0, Row0);
	R.SetRow(1, Row1);
	R.SetRow(2, Row2);
	return R;
}
Matrix Euler_Rotation::GetRz(const float& ang) const {
	Vector Row0(cosf(ang), sinf(ang), 0.f);
	Vector Row1(-sinf(ang), cosf(ang), 0.f);
	Vector Row2(0.f, 0.f, 1.f);
	Matrix R;
	R.SetRow(0, Row0);
	R.SetRow(1, Row1);
	R.SetRow(2, Row2);
	return R;
}
Matrix Euler_Rotation::GetRotationMatrix( /* based on https://mathworld.wolfram.com/EulerAngles.html */
	const float& phi,
	const float& theta,
	const float& psi
) const {
	/* R_z(psi) * R_x(theta) * R_z(phi) */
	Vector Row0(
		(cosf(psi) * cosf(phi) - cosf(theta) * sinf(phi) * sinf(psi)), 
		(cosf(psi) * sinf(phi) + cosf(theta) * cosf(phi) * sinf(psi)),
		(sinf(psi)*sinf(theta))
	);
	Vector Row1(
		(-sinf(psi) * cosf(phi) - cosf(theta) * sinf(phi) * cosf(psi)),
		(-sinf(psi) * sinf(phi) + cosf(theta) * cosf(phi) * cosf(psi)),
		(cosf(psi) * sinf(theta))
	);
	Vector Row2(
		(sinf(theta) * sinf(phi)),
		(-sinf(theta) * cosf(phi)),
		cosf(theta)
	);
	Matrix R;
	R.SetRow(0, Row0);
	R.SetRow(1, Row1);
	R.SetRow(2, Row2);
	return R;
}
Matrix Euler_Rotation::GetInvRotationMatrix(
	const float& phi,
	const float& theta,
	const float& psi
) const {
	/*  R_x(-phi) * R_x(-theta) * R_z(-psi) */
	return GetRotationMatrix(-psi, -theta, -phi);
}
void Euler_Rotation::GetRotationAngles(
	const Matrix& R,
	float& phi,
	float& theta,
	float& psi
) const {

	Vector Row0 = R.GetRow(0);
	Vector Row1 = R.GetRow(1);
	Vector Row2 = R.GetRow(2);
	float cos_theta, cos_phi, sin_phi;
	cos_theta = Row2.Get_i(2);
	theta = acosf(cos_theta);
	/* gimbal lock if theta = 0 
	   if theta = 0 then there was no rotation about x
	   and the two rotations about z are ambiguous 
	   */
	if (theta <= 0.f) {/*really theta=0*/
		/*Row0		cos(psi + phi), sin(psi+phi), 0 */
		psi = 0.f;
		cos_phi = Row0.Get_i(0);
		sin_phi = Row0.Get_i(1);
	}
	else if (theta >= PI_f) {/*really theta=pi*/
		psi = 0.f;
		cos_phi = Row0.Get_i(0);
		sin_phi = -Row0.Get_i(1);
	}
	else {
		float sin_theta = sqrtf(1 - cos_theta * cos_theta);/*theta is [0,pi] so sin(theta) is always >=0 */
		float sin_psi = Row0.Get_i(2) / sin_theta;
		float cos_psi = Row1.Get_i(2) / sin_theta;
		psi = Math2D::Sin_And_Cos_To_Angle(sin_psi, cos_psi);

		sin_phi = Row2.Get_i(0) / sin_theta;
		cos_phi = Row2.Get_i(1) / (-sin_theta);
	}
	phi = Math2D::Sin_And_Cos_To_Angle(sin_phi, cos_phi);
}
void Euler_Rotation::Rotate(
	const float& phi,
	const float& theta,
	const float& psi,
	Vector& v
) const {
	Matrix R = GetRotationMatrix(phi, theta, psi);
	v = R.Mul(v);
}