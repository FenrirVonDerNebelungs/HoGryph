#include "Axis.h"
Axis_Angle::Axis_Angle() {
	;
}
Axis_Angle::Axis_Angle(const Vector& v) : Vector(v) {
	this->Normalize();
}
Axis_Angle::Axis_Angle(const Vector& u, float angle) : Vector(u) {
	this->Normalize();
	this->Set_Scalar(angle);
}
Axis_Angle::~Axis_Angle() {
	;
}
void Axis_Angle::SetAxis(const Vector& u) {
	for (int i = 0; i < 3; i++)
		this->m_vec[i] = u.Get_i(i);
	this->Normalize();
}
void Axis_Angle::SetAngle(const float& angle) {
	this->m_scalar = angle;
}
Vector Axis_Angle::GetAxis() const {
	Vector u(*this);
	return u;
}
float Axis_Angle::GetAngle()const {
	return m_scalar;
}

AxisRotation::AxisRotation() {
	;
}
AxisRotation::~AxisRotation() {
	;
}
Axis_Angle AxisRotation::GetFlatRotationAxis(
	const Vector& v,
	const Vector& v_new
) const {
	Vector V_axis = v.Cross(v_new);
	/* a X b = |a||b|sin(ang) */
	float V_axis_len = V_axis.Vector_Length();
	float v_len = v.Vector_Length();
	float v_new_len = v_new.Vector_Length();
	if (v_len <= 0.f || v_new_len <= 0.f) {
		Axis_Angle Null_Axis(v);
		Null_Axis.Normalize();
		return Null_Axis;
	}
	float v_mul_len = v_len * v_new_len;
	float sin_term = V_axis_len/v_mul_len;
	float cos_term = v.Dot(v_new)/v_mul_len;
	float angle = Math2D::Sin_And_Cos_To_Angle(sin_term, cos_term);
	Axis_Angle AngNVec(V_axis);
	AngNVec.SetAngle(angle);
	return AngNVec;
}
Vector AxisRotation::Rotate(
	const Axis_Angle& rotation_about_axis,
	const Vector& v
) const {
	Vector rotated_v(v);

	Vector u(rotation_about_axis);
	u.Normalize();
	float a = rotation_about_axis.Get_Scalar();
	float s1 = cosf(a);
	float s2 = (1 - s1) * (u.Dot(v));
	float s3 = sinf(a);
	Vector term1 = v * s1;
	Vector term2 = u * s2;
	Vector vector_part_term3 = u.Cross(v);
	if (vector_part_term3.Vector_Length() != 0.f && a != 0.f) {
		Vector term3 = vector_part_term3 * s3;
		rotated_v = term1 + term2 + term3;
	}
	return rotated_v;
}
Matrix AxisRotation::GetRotationMatrix(
	const Vector& new_X,
	const Vector& new_Y,
	const Vector& new_Z
) const {
	Matrix M_Rot;
	M_Rot.SetCol(0, new_X);
	M_Rot.SetCol(1, new_Y);
	M_Rot.SetCol(2, new_Z);
	return M_Rot;
}
void AxisRotation::GetAxesFromRotation(
	const Matrix& R,
	Vector& new_X,
	Vector& new_Y,
	Vector& new_Z
) const {
	new_X = R.GetCol(0);
	new_Y = R.GetCol(1);
	new_Z = R.GetCol(2);
}
Matrix AxisRotation::GetRotationMatrix(const Axis_Angle& u_ang) const {
	Vector X_axis(1.f, 0.f, 0.f);
	Vector X_axis_new = Rotate(u_ang, X_axis);
	Vector Y_axis(0.f, 1.f, 0.f);
	Vector Y_axis_new = Rotate(u_ang, Y_axis);
	Vector Z_axis(0.f, 0.f, 1.f);
	Vector Z_axis_new = Rotate(u_ang, Z_axis);
	return GetRotationMatrix(X_axis_new, Y_axis_new, Z_axis_new);
}
Axis_Angle AxisRotation::GetRotation(const Matrix& M_Rot)const {
	/* if a vector is colinear with the rotation axis it will be invariant under multiplication by M_Rot 
	*  the rotation axis is an eigenvector with eigenvalue 1 of M_Rot
	*/
	float angle = 0.f;
	Vector rotation_axis = M_Rot.GetEigenVector(1.f);
	if (rotation_axis.Vector_Length() > 0.99f) {
		Vector X(1.f, 0.f, 0.f);
		Vector Y(0.f, 1.f, 0.f);
		Vector Z(0.f, 0.f, 1.f);
		Vector rot_X;
		Vector rot_Y;
		Vector rot_Z;
		GetAxesFromRotation(M_Rot, rot_X, rot_Y, rot_Z);
		if (rotation_axis.Dot(rot_X) < 0.95f) {
			angle = GetRotationAngle(rotation_axis, X, rot_X);
		}
		else if (rotation_axis.Dot(rot_Y) < 0.95f) {
			angle = GetRotationAngle(rotation_axis, Y, rot_Y);
		}
	}
	Axis_Angle rotation_axis_angle(rotation_axis);
	rotation_axis_angle.SetAngle(angle);
	return rotation_axis_angle;
}
float AxisRotation::GetRotationAngle(
	const Vector& rotation_axis,
	const Vector& v,
	const Vector& v_new
) const {
	Vector u_perp = v.Cross(rotation_axis);
	u_perp.Normalize();
	Vector u_new_perp = v_new.Cross(rotation_axis);
	u_new_perp.Normalize();
	/* a X b =|a||b|sin(theta)
	*  a * b = |a||b| cos(theta)
	*/
	Vector sin_term_vec = u_perp.Cross(u_new_perp);
	float sin_term = sin_term_vec.Vector_Length();
	if (sin_term_vec.Dot(rotation_axis) < 0.f)
		sin_term *= -1.f;
	float cos_term = u_perp.Dot(u_new_perp);
	float rot_angle = Math2D::Sin_And_Cos_To_Angle(sin_term, cos_term);
	return rot_angle;
}
/*
Axis_Angle AxisRotation::GetAxisRotationFromMatrix(Matrix& M_Rot) const{
	Vector X(1.0f, 0.f, 0.f);
	Vector Y(0.f, 1.f, 0.f);
	Vector Z(0.f, 0.f, 1.f);
	Vector Rot_X, Rot_Y, Rot_Z;
	GetAxesFromRotation(M_Rot, Rot_X, Rot_Y, Rot_Z);
	Axis_Angle rot2_u_ang = GetFlatRotationAxis(Z, Rot_Z);
	Matrix M_Rot2 = GetRotationMatrix(rot2_u_ang);
	Matrix M_Rot2_inv = GetInvRotationMatrix(rot2_u_ang);
	Vector Rot2_X = M_Rot2_inv * Rot_X;
	Axis_Angle rot1_u_ang = GetFlatRotationAxis(X, Rot2_X);
	Matrix M_Rot1 = GetRotationMatrix(rot1_u_ang);
	
}*/