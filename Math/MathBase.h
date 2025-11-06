#pragma once
#include <cmath>
#include <cfloat>

#ifndef MATHBASE_H
#define MATHBASE_H
const float PI_f = 2.f * sinf(1.f);

class Quaternion;

namespace Math2D {
	float Sin_And_Cos_To_Angle(const float& sin_ang, const float& cos_ang);/* finds the 2PI angle for a given sin and cos value
																			uses asinf output range -pi/2, pi/2 
																			or acosf output range 0 to pi 
																			*/
}
class _2Tuple {
public:
	_2Tuple();
	_2Tuple(float x, float y);
	~_2Tuple();

protected:
	float m_vec[2];
};
class _3Tuple {
public:
	_3Tuple();
	_3Tuple(float x, float y, float z);
	_3Tuple(const _3Tuple& other);
	~_3Tuple();

	_3Tuple& operator=(const _3Tuple& other);

	inline void Set_Scalar(float s) { m_scalar = s; }
	void Set(float x, float y, float z);
	void Set_i(int i, float val);
	virtual void Equal(const _3Tuple& other);

	inline float Get_Scalar() const { return m_scalar; }
	void Get(float vec[]) const;
	float Get_i(int i) const;

	_3Tuple Add(const _3Tuple& other) const;
	_3Tuple Sub(const _3Tuple& other) const;
	_3Tuple Mul(const float& scalar) const;
	virtual inline void Mul(const float& scalar) { for (int i = 0; i < 3; i++) this->m_vec[i] *= scalar; }

	virtual void Normalize();/*set the magnitude of the vector created by this 3Tuple to 1*/
	float Vector_Length() const;/*vector length*/
protected:
	float m_scalar;
	float m_vec[3];
};
class Vector : public _3Tuple{
public:
	Vector();
	Vector(float x, float y, float z);
	Vector(const Vector& other);
	Vector(const Quaternion& other);/*this will not preserve the scalar*/
	~Vector();

	bool operator== (const Vector& other) const;
	bool operator!= (const Vector& other);
	Vector& operator=(const Vector& rhs);
	float operator* (const Vector& rhs) const;
	Vector operator* (const float& rhs) const;
	Vector operator+ (const Vector& rhs) const;

	Vector Mul(const float& rhs) const;
	float Dot(const Vector& vec) const;
	Vector Cross(const Vector& vec) const;
protected:
	friend class Matrix;
	void Switch_Elements(const int& el1_i, const int& el2_i);
};

class Matrix {
public:
	Matrix();
	Matrix(const Matrix& other);

	void SetRow(int row_i, const Vector& vec);
	void SetCol(int col_j, const Vector& vec);
	inline void Set_ij(int i, int j, float val) { int indx = i + j * 3; m_val[indx] = val; }
	Matrix& operator=(const Matrix& rhs);
	Matrix operator*(const Matrix& rhs) const;
	Vector operator*(const Vector& rhs) const;
	void Equal(const Matrix& other);

	inline float Get_ij(int i, int j) const { int indx = i + j * 3; return m_val[indx]; }
	Vector GetRow(int row_i) const;
	Vector GetCol(int col_j) const;

	Matrix Mul(const Matrix& other) const;
	Vector Mul(const Vector& other) const;
	Matrix Mul(const float& scalar) const;

	float Det() const; /* determinate of matrix */
	Matrix CofactorMatrix() const;
	Matrix Transpose() const;
	Matrix Inverse() const;

	/*Vector SolveLinearEquation(const Vector& B) const;*/ /*finds X from MX=B*/

	inline float GetValFromInteralIndex(int i) const { return m_val[i]; }
protected:
	float m_val[9];
	float Epsilon_ijk(const int& i, const int& j, const int& k) const; /*Levi-Civita symbol*/
	float Cofactor(const Matrix& other, const int& i1, const int& i2) const;
	void AddRow(const int& row_i, const Vector& row_to_add, const float& row_multiple = 1.f);
	void AddRows(const int& row_pulled_to_add_i, const float& row_pulled_multiple, const int& row_recieving_add_i);

	void Switch_Rows(const int& row1_i, const int& row2_i);
	void Switch_Extended_Rows(
		const int& row1_i,
		const int& row2_i,
		Matrix& M,
		Vector& V) const;
	void MulRow(const int& i, const float& a);/*multiply row i by a*/
};
#endif