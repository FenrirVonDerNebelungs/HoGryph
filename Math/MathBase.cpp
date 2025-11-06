#include "MathBase.h"
#include "../Rotations/Quaternion.h"

float Math2D::Sin_And_Cos_To_Angle(const float& sin_ang, const float& cos_ang) {
	float ang = acosf(cos_ang);/* returns value from 0 to pi */
	if (sin_ang < 0.f) {
		if (ang < PI_f)
			ang += PI_f;
		else
			ang = 0.f;
	}
	return ang;
}

_2Tuple::_2Tuple() {
	m_vec[0] = 0.f;
	m_vec[1] = 0.f;
}
_2Tuple::_2Tuple(float x, float y) {
	m_vec[0] = x;
	m_vec[1] = y;
}
_2Tuple::~_2Tuple() {
	;
}
_3Tuple::_3Tuple():m_scalar(0) {
	m_vec[0] = 0.f;
	m_vec[1] = 0.f;
	m_vec[2] = 0.f;
}
_3Tuple::_3Tuple(float x, float y, float z):m_scalar(0) {
	Set(x, y, z);
}
_3Tuple::_3Tuple(const _3Tuple& other) {
	Equal(other);
}
_3Tuple::~_3Tuple() {
	;
}
_3Tuple& _3Tuple::operator=(const _3Tuple& other) {
	Equal(other);
	return *this;
}

void _3Tuple::Set(float x, float y, float z) {
	m_vec[0] = x;
	m_vec[1] = y;
	m_vec[2] = z;
}
void _3Tuple::Set_i(int i, float val) {
	if (i < 0 || i >= 3)
		return;
	m_vec[i] = val;
}
void _3Tuple::Equal(const _3Tuple& other) {
	for (int i = 0; i < 3; i++)
		this->m_vec[i] = other.Get_i(i);
	this->m_scalar = other.m_scalar;
}
void _3Tuple::Get(float vec[]) const {
	for (int i = 0; i < 3; i++)
		vec[i] = m_vec[i];
}
float _3Tuple::Get_i(int i) const{
	if (i < 0 || i >= 3)
		return 0.f;
	return m_vec[i];
}
_3Tuple _3Tuple::Add(const _3Tuple& other)const {
	_3Tuple sum;
	for (int i = 0; i < 3; i++)
		sum.Set_i(i, (this->Get_i(i) + other.Get_i(i)));
	return sum;
}
_3Tuple _3Tuple::Sub(const _3Tuple& other) const {
	_3Tuple diff;
	for (int i = 0; i < 3; i++)
		diff.Set_i(i, (this->Get_i(i) - other.Get_i(i)));
	return diff;
}
_3Tuple _3Tuple::Mul(const float& scalar) const {
	_3Tuple mulTup(*this);
	for (int i = 0; i < 3; i++)
		mulTup.Set_i(i, (mulTup.Get_i(i) * scalar));
	return mulTup;
}
void _3Tuple::Normalize() {
	float vector_len = Vector_Length();
	if (vector_len > 0.f)
		for (int i = 0; i < 3; i++)
			this->m_vec[i] /= vector_len;
}
float _3Tuple::Vector_Length() const{
	float len = 0;
	for (int i = 0; i < 3; i++)
		len += this->m_vec[i] * this->m_vec[i];
	return sqrtf(len);
}
Vector::Vector() {
	;
}
Vector::Vector(float x, float y, float z) : _3Tuple(x, y, z) {
	;
}
Vector::Vector(const Vector& other) : _3Tuple(other) {
	;
}
Vector::Vector(const Quaternion& other) {
	for (int i = 0; i < 3; i++)
		this->m_vec[i] = other.Get_i(i);
}
Vector::~Vector() {
	;
}
bool Vector::operator==(const Vector& other) const{
	for (int i = 0; i < 3; i++)
		if (this->m_vec[i] != other.m_vec[i])
			return false;
}
bool Vector::operator!=(const Vector& other) {
	for (int i = 0; i < 3; i++)
		if (this->m_vec[i] != other.Get_i(i))
			return true;
}
Vector& Vector::operator=(const Vector& rhs) {
	_3Tuple::Equal(rhs);
	return *this;
}
float Vector::operator*(const Vector& rhs) const{
	return this->Dot(rhs);
}
Vector Vector::operator*(const float& rhs) const{
	return Mul(rhs);
}
Vector Vector::operator+(const Vector& rhs) const {
	Vector new_v(*this);
	for (int i = 0; i < 3; i++) {
		float new_term = this->Get_i(i) + rhs.Get_i(i);
		new_v.Set_i(i, new_term);
	}
	return new_v;
}
Vector Vector::Mul(const float& rhs) const{
	Vector new_v(*this);
	for (int i = 0; i < 3; i++) {
		float new_ith_term = rhs*(this->Get_i(i));
		new_v.Set_i(i,new_ith_term);
	}
	return new_v;
}
float Vector::Dot(const Vector& vec) const {
	float dotprod = 0.f;
	for (int i = 0; i < 3; i++)
		dotprod += this->m_vec[i] * vec.Get_i(i);
	return dotprod;
}
Vector Vector::Cross(const Vector& vec) const {
	float v0 = m_vec[1] * vec.Get_i(2) - m_vec[2] * vec.Get_i(1);
	float v1 = m_vec[2] * vec.Get_i(0) - m_vec[0] * vec.Get_i(2);
	float v2 = m_vec[0] * vec.Get_i(1) - m_vec[1] * vec.Get_i(0);
	Vector crossprod(v0, v1, v2);
	return crossprod;
}
void Vector::Switch_Elements(const int& el1, const int& el2) {
	float el_switch = m_vec[el1];
	m_vec[el1] = m_vec[el2];
	m_vec[el2] = el_switch;
}
Matrix::Matrix() {
	for (int i = 0; i < 9; i++)
		m_val[i] = 0.f;
}
Matrix::Matrix(const Matrix& other) {
	Equal(other);
}

void Matrix::SetRow(int row_i, const Vector& vec) {
	for (int j = 0; j < 3; j++)
		Set_ij(row_i, j, vec.Get_i(j));
}
void Matrix::SetCol(int col_j, const Vector& vec) {
	for (int i = 0; i < 3; i++)
		Set_ij(i, col_j, vec.Get_i(i));
}
Matrix& Matrix::operator=(const Matrix& rhs) {
	this->Equal(rhs);
	return *this;
}
Matrix Matrix::operator*(const Matrix& rhs) const{
	Matrix M_Mul = this->Mul(rhs);
	return M_Mul;
}
Vector Matrix::operator*(const Vector& rhs) const {
	Vector V_Mul = this->Mul(rhs);
	return V_Mul;
}
void Matrix::Equal(const Matrix& other) {
	for (int i = 0; i < 9; i++)
		m_val[i] = other.GetValFromInteralIndex(i);
}



Vector Matrix::GetRow(int row_i) const {
	Vector Row_vec;
	for (int j = 0; j < 3; j++)
		Row_vec.Set_i(j, this->Get_ij(row_i, j));
	return Row_vec;
}
Vector Matrix::GetCol(int col_j) const {
	Vector Col_vec;
	for (int i = 0; i < 3; i++)
		Col_vec.Set_i(i, this->Get_ij(i, col_j));
	return Col_vec;
}
Matrix Matrix::Mul(const Matrix& other) const {
	Matrix multiplied_result;
	for (int col_j = 0; col_j < 3; col_j++) {
		Vector other_column_vec = other.GetCol(col_j);
		Vector multiplied_column = this->Mul(other_column_vec);
		multiplied_result.SetCol(col_j, multiplied_column);
	}
	return multiplied_result;
}
Vector Matrix::Mul(const Vector& other) const{
	Vector multiplied_result;
	for (int row_i = 0; row_i < 3; row_i++) {
		float row_sum = 0.f;
		for (int col_j = 0; col_j < 3; col_j++) {
			row_sum += this->Get_ij(row_i, col_j) * other.Get_i(col_j);
		}
		multiplied_result.Set_i(row_i, row_sum);
	}
	return multiplied_result;
}
Matrix Matrix::Mul(const float& scalar) const {
	Matrix multiplied_matrix(*this);
	for (int i = 0; i < 9; i++)
		multiplied_matrix.m_val[i] *= scalar;
	return multiplied_matrix;
}
float Matrix::Det() const {
	Vector top_row = this->GetRow(0);
	float det_sum = 0.f;
	for (int c = 0; c < 3; c++) {
		float a_0c = top_row.Get_i(c);
		float a_cofactor = Cofactor(*this, 0, c);
		det_sum += a_0c * a_cofactor;
	}
	return det_sum;
}
Matrix Matrix::CofactorMatrix() const {
	Matrix cofactor_matrix;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			float cofactor_rc = Cofactor(*this, r, c);
			cofactor_matrix.Set_ij(r, c, cofactor_rc);
		}
	}
	return cofactor_matrix;
}
Matrix Matrix::Transpose() const {
	Matrix trans_matrix;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float a_ij = this->Get_ij(i, j);
			trans_matrix.Set_ij(j, i, a_ij);
		}
	}
	return trans_matrix;
}
Matrix Matrix::Inverse() const {
	Matrix cofactor_M = CofactorMatrix();
	Matrix cofactor_trans_M = cofactor_M.Transpose();
	float det_M = Det();
	if (det_M == 0.f)
		return Matrix();
	Matrix inverse_M = cofactor_trans_M.Mul(1.f / det_M);
	return inverse_M;
}
/*
Vector Matrix::SolveLinearEquation(const Vector& B) const {
	Vector V(B);
	Matrix M(*this);
	for (int i = 0; i< 3; i++) {
		float a_i0 = M.Get_ij(i, 0);
		if (a_i0 != 0.f) {
			M.MulRow(i, 1.f / a_i0);
			if (i != 0) {
				Switch_Extended_Rows(0, i, M, V);
			}
			if (a_i0 < 2) {
				for (int ii = 1; ii < 3; ii++) {
					float a_ii_0 = M.Get_ij(ii, 0);
					if (a_ii_0 != 0.f) {
						float elim_factor = -a_ii_0;
						M.AddRows(0, elim_factor, a_ii_0);
					}
				}
			}
			break;
		}
	}
	for (int i = 1; i < 3; i++) {
		float a_i1 = M.Get_ij(i, 1);
		if (a_i1 != 0.f) {
			M.MulRow(i, 1.f / a_i1);
			if (i != 1) {
				Switch_Extended_Rows(1, i, M, V);
			}
		}

	}

}
*/
float Matrix::Epsilon_ijk(const int& i, const int& j, const int& k) const{
	if (i == j || i == k || j == k)
		return 0.f;
	int n_i = i;
	int n_j = j;
	int n_k = k;
	float sign_ = 1.f;
	if (n_j < n_i) {
		sign_ = -1.f;
		n_j = i;
		n_i = j;
	}
	if (n_k < n_i) {
		sign_ *= -1.f;
		n_k = n_i;
		n_i = k;
	}
	if (n_k < n_j) {
		sign_ *= -1.f;
		int o_j = n_j;
		n_k = n_j;
		n_j = o_j;
	}
	return sign_;
}
float Matrix::Cofactor(const Matrix& other, const int& r, const int& c) const {
	/* Determinate = 1/(3!) Sum_r1 Sum_r2 Sum_r3 Sum_c1 Sum_c2 Sum_c3 epsilon_r1r2r3 epsilon_c1c2c3 a_r1c1 a_r2c2 a_r3c3
	   Sum_r1 Sum_r2 Sum_r3 Sum_c1 Sum_c2 Sum_c3 epsilon_r1r2r3 epsilon_c1c2c3 = n! */
	float cofactor_sum = 0.f;
	for (int r1 = 0; r1 < 3; r1++) {
		if (r1 == r)
			continue;
		for (int r2 = 0; r2 < 3; r2++) {
			if (r2 == r)
				continue;
			for (int c1 = 0; c1 < 3; c1++) {
				if (c1 == c)
					continue;
				for (int c2 = 0; c2 < 3; c2++) {
					if (c2 == c)
						continue;
					float epsilon_1 = Epsilon_ijk(r, r1, r2);
					float epsilon_2 = Epsilon_ijk(c, c1, c2);
					float a_r1c1 = other.Get_ij(r1, c1);
					float a_r2c2 = other.Get_ij(r2, c2);
					cofactor_sum += epsilon_1 * epsilon_2 * a_r1c1 * a_r2c2;
				}
			}
		}
	}
	cofactor_sum /= 2.f;/* 1/2! term for the small det for each cofactor */
	return cofactor_sum;
}
void Matrix::AddRow(
	const int& row_i,
	const Vector& row_to_add,
	const float& row_multiple
) {
	Vector ith_row = this->GetRow(row_i);
	Vector row_to_add_scaled = row_to_add * row_multiple;
	Vector new_ith_row = ith_row + row_to_add_scaled;
	this->SetRow(row_i, new_ith_row);
}
void Matrix::AddRows(
	const int& row_pulled_to_add_i,
	const float& row_pulled_multiple,
	const int& row_recieving_add_i
) {
	Vector row_pulled = this->GetRow(row_pulled_to_add_i);
	AddRow(row_pulled_to_add_i, row_pulled, row_pulled_multiple);
}
void Matrix::Switch_Rows(const int& row1_i, const int& row2_i) {
	Vector switch_row = this->GetRow(row1_i);
	this->SetRow(row1_i, this->GetRow(row2_i));
	this->SetRow(row2_i, switch_row);
}
void Matrix::Switch_Extended_Rows(
	const int& row1_i,
	const int& row2_i,
	Matrix& M,
	Vector& V
) const {
	M.Switch_Rows(row1_i, row2_i);
	V.Switch_Elements(row1_i, row2_i);
}
void Matrix::MulRow(const int& i, const float& a) {
	Vector row_i = this->GetRow(i);
	Vector row_mult_i = row_i * a;
	this->SetRow(i, row_mult_i);
}