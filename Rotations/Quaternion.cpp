#include "Quaternion.h"

Quaternion::Quaternion() {
	;
}
Quaternion::Quaternion(float x, float y, float z, float scalar) : _3Tuple(x, y, z) {
	m_scalar = scalar;
}
Quaternion::Quaternion(const Quaternion& other) : _3Tuple(other) {
	;
}
Quaternion::Quaternion(const _3Tuple& other) : _3Tuple(other) {
	;
}
Quaternion::Quaternion(const Vector& other, float scalar) :_3Tuple(other) {
	this->m_scalar = other.Get_Scalar();
}
Quaternion::~Quaternion() {
	;
}
Quaternion& Quaternion::operator* (const Quaternion& rhs) {
	Quaternion multiplicaton_result = Mul(rhs);
	this->Equal(multiplicaton_result);
	return *this;
}
Quaternion Quaternion::operator+ (const Quaternion& rhs) const{
	Quaternion sum = this->Add(rhs);
	sum.Set_Scalar(this->m_scalar + rhs.m_scalar);
	return sum;
}
void Quaternion::Equal(const Quaternion& other) {
	_3Tuple::Equal(other);
}
void Quaternion::Inv() {
	for (int i = 0; i < 3; i++)
		this->m_vec[i] = -(this->m_vec[i]);
}
Quaternion Quaternion::Mul(const Quaternion& other) const {
	/* q1 q2  
	* (q1_s + q1_vec)(q2_s + q2_vec)
	* q1_s * q2_s + q1_s(q2_vec) + (q1_vec)q2_s + (q1_vec)(q2_vec)
	* q1_s * q2_s + q1_s(q2_vec) + q2_s(q1_vec) + (q1_vec)(q2_vec)
	*/
	/* q1_s*q2_s */
	Quaternion term1(0.f, 0.f, 0.f, this->m_scalar * other.m_scalar);
	/*q1_s(q2_vec)*/
	Quaternion term2 = VectorPartOnly(other);
	term2.Mul_Vector_Part(this->m_scalar);
	/*q2_s(q1_vec)*/
	Quaternion term3 = VectorPartOnly(*this);
	term3.Mul_Vector_Part(other.m_scalar);
	/*(q1_vec)(q2_vec)*/
	Quaternion term4 = this->MulVectorParts(other);
	return term1 + term2 + term3 + term4;
}
Quaternion Quaternion::MulVectorParts(const Quaternion& other) const{
	Quaternion multiplied_quat;
	Quaternion_component all_multipication_terms[9];
	int cnt_multiplication_terms = 0;
	for (int i_this = 0; i_this < 3; i_this++) {
		Quaternion_component this_term;
		this_term.i = i_this;
		this_term.val = this->Get_i(i_this);
		for (int i_other = 0; i_other < 3; i_other++) {
			Quaternion_component other_term;
			other_term.i = i_other;
			other_term.val = other.Get_i(i_other);
			all_multipication_terms[cnt_multiplication_terms] = component_multiplication(this_term, other_term);
			cnt_multiplication_terms++;
		}
	}
	/*add the scalar part*/
	float scalar_value = 0.f;
	for (int i = 0; i < 9; i++) {
		if (all_multipication_terms[i].i == 3 /*3 is coded as beyond the highest index, and indicate a scalar*/)
			scalar_value += all_multipication_terms[i].val;
	}
	multiplied_quat.Set_Scalar(scalar_value);
	/*add the vector part*/
	for (int i = 0; i < 9; i++) {
		multiplied_quat.add_component_to_this_quaternion(all_multipication_terms[i]);
	}
	return multiplied_quat;
}
void Quaternion::Normalize() {
	this->m_scalar = 0.f;
	_3Tuple::Normalize();
}
Quaternion_component Quaternion::component_multiplication(const Quaternion_component& this_component, const Quaternion_component& other_component) const {
	Quaternion_component mul_quats;
	mul_quats.val = this_component.val * other_component.val;
	mul_quats.i = 3;/* exceeds allowed return indexes, indicating scalar */
	/*start by finding the indexes, by defaul the indexes will start from zero but this needs to be reset to 1*/
	int this_index = this_component.i + 1;
	int other_index = other_component.i + 1;
	int mul_index = component_index_multiplication(this_index, other_index);
	if (mul_index == 0) {
		mul_quats.val *= -1.f;
	}
	else {

		if (mul_index < 0) {
			mul_quats.val *= -1.f;
		}
		mul_quats.i = (std::abs(mul_index) - 1);

	}

	return mul_quats;
}
int Quaternion::component_index_multiplication(int this_component_index, int other_component_index) const {
	/*
	* i=0, j=1, k=2
	* ii=jj=kk=-1
	* ij=k=-ji
	* jk=i=-kj
	* ki=j=-ik
	*/

	if (this_component_index == other_component_index)
		return 0;
	int sign = 1;
	if (other_component_index < this_component_index) 
		sign *= -1;
	if (std::abs(other_component_index - this_component_index) >= 2)
		sign *= -1;
	
	int available_component_indexes[] = { 1, 2, 3 };
	for (int i = 0; i < 3; i++) {
		if (other_component_index == available_component_indexes[i] || this_component_index == available_component_indexes[i])
			available_component_indexes[i] = -1;
	}
	int ret_index = 0;
	for (int i = 0; i < 3; i++)
		if (available_component_indexes[i] > 0) {
			ret_index = available_component_indexes[i];
			break;
		}
	return sign * ret_index;
}
void Quaternion::add_component_to_this_quaternion(const Quaternion_component& val) {
	if (val.i < 0 || val.i >= 3)
		return;
	this->m_vec[val.i] += val.val;
}
Quaternion Quaternion::VectorPartOnly(const Quaternion& q) const{
	Quaternion vec(q);
	vec.Set_Scalar(0.f);
	return vec;
}

Quaternion_Rotation::Quaternion_Rotation() {
	;
}
Quaternion_Rotation::~Quaternion_Rotation() {
	;
}
Quaternion Quaternion_Rotation::GetRotationQuaternion(const Axis_Angle& Ang_N_Vec) const {
	Quaternion u(Ang_N_Vec);
	float a = Ang_N_Vec.Get_Scalar();
	u.Set_Scalar(0.f);
	return rotationQuaternion(a, u);
}
Axis_Angle Quaternion_Rotation::GetRotationAxis(const Quaternion& q) const {
	Axis_Angle ang_vec;
	float half_angle = acosf(q.Get_Scalar());
	ang_vec.Set_Scalar(half_angle * 2.f);
	Vector vector_part(q);
	float vector_part_magnitude = q.Vector_Length();
	Vector unit_vector_part;
	if (vector_part_magnitude >= 0) {
		unit_vector_part = vector_part.Mul(1.f / vector_part_magnitude);
	}
	ang_vec.Set(unit_vector_part.Get_i(0), unit_vector_part.Get_i(1), unit_vector_part.Get_i(2));
	return ang_vec;
}
Matrix Quaternion_Rotation::GetRotationMatrix(const Quaternion& q) const {
	Axis_Angle u_ang = GetRotationAxis(q);
	return AxisRotation::GetRotationMatrix(u_ang);
}
Vector Quaternion_Rotation::Rotate(const Axis_Angle& rotation_about_axis, const Vector& v) const{
	const Vector u(rotation_about_axis);
	float a = rotation_about_axis.Get_Scalar();
	Quaternion q = rotationQuaternion(a, u);
	return Rotate(q, v);
}
Vector Quaternion_Rotation::Rotate(const Quaternion& q, const Vector& v) const {
	Vector new_v(v);
	Quaternion q_inv(q);
	q_inv.Inv();
	Quaternion mul_qv = q.Mul(v);
	Quaternion rotated_v = mul_qv.Mul(q_inv);
	new_v.Set(rotated_v.Get_i(0), rotated_v.Get_i(1), rotated_v.Get_i(2));
	return new_v;
}

Quaternion Quaternion_Rotation::CombineRotation(const Quaternion& q1, const Quaternion& q2) const {
	Quaternion q_rot = q2.Mul(q1);
	return q_rot;
}
Quaternion Quaternion_Rotation::CombineRotation(const Quaternion& q1, const Quaternion& q2, const Quaternion& q3) const {
	Quaternion q2_q1 = CombineRotation(q1, q2);
	Quaternion q_rot = q3.Mul(q2_q1);
	return q_rot;
}


Quaternion Quaternion_Rotation::GetRotation(const Matrix& R) const {
	AxisRotation Axis_Rotar;

	Vector X_new = R.GetCol(0);
	Vector Z_new = R.GetCol(2);
	Vector X(1.f, 0.f, 0.f);
	Vector Z(0.f, 0.f, 1.f);
	Vector X_in_plane;

	Quaternion q2(0.f, 0.f, 1.f, 0.f);
	if (Z_new != Z) {
		/*find how much Z rotated*/

		Axis_Angle Z_Axis_Spinner = Axis_Rotar.GetFlatRotationAxis(Z, Z_new);/* u is in direction Z X Z_new */
	    q2 = GetRotationQuaternion(Z_Axis_Spinner);
		Quaternion q2_inv(q2);
		q2_inv.Inv();
		/*rotate the Z axis back into it's original position, and find out how this transforms the x axis*/
		X_in_plane = Rotate(q2_inv, X_new);
	}
	else {
		/* gimbal lock, so rotate only about phi */
		X_in_plane = X_new;
	}
	Axis_Angle X_Axis_Spinner = Axis_Rotar.GetFlatRotationAxis(X, X_in_plane);
	Quaternion q1 = GetRotationQuaternion(X_Axis_Spinner);

	Quaternion q = CombineRotation(q1, q2);

	return q;
}


Quaternion Quaternion_Rotation::rotationQuaternion(float a, const Quaternion& u) const{
	Quaternion u_norm(u);
	u_norm.Normalize();
	float q_0 = cosf(a / 2.f);
	float q_vec_mul = sinf(a / 2.f);

	Quaternion q(u_norm);
	q.Mul_Vector_Part(q_vec_mul);
	q.Set_Scalar(q_0);
	return q;
}
void Quaternion_Rotation::Rotate(const Vector& u, float& a, Vector& v) const {
	Axis_Angle rot_axis(u);
	rot_axis.SetAngle(a);
	v = Rotate(rot_axis, v);
}