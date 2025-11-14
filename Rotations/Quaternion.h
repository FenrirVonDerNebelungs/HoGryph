#pragma once
#ifndef QUATERNION_H
#define QUATERNION_H

#ifndef AXIS_H
#include "Axis.h"
#endif

struct Quaternion_component {
	float val;
	int i;
};
class Quaternion : public _3Tuple {/*really 3 quaternion*/
public:
	Quaternion();
	Quaternion(float x, float y, float z, float scalar);
	Quaternion(const Quaternion& other);
	Quaternion(const _3Tuple& other);
	Quaternion(const Vector& other, float scalar=0.f);
	~Quaternion();

	Quaternion& operator* (const Quaternion& rhs);
	Quaternion operator+ (const Quaternion& rhs) const;
	void Equal(const Quaternion& other);
	void Inv();/*invert this quaternion*/

	Quaternion Mul(const Quaternion& other) const;
	inline void Mul_Vector_Part(const float& scalar) { _3Tuple::Mul(scalar); }

	void Normalize();/*sets this quaternion's scalar to 0 and vector to 1*/
protected:
	Quaternion MulVectorParts(const Quaternion& other) const; /*ignores scalar values for this & other */
	Quaternion_component component_multiplication(const Quaternion_component& this_component, const Quaternion_component& other_component) const;/*indexes are from 0 to 2*/
	int component_index_multiplication(int this_component_index, int other_component_index) const;  /*indexes need to start at 1 and go to 3*/

	void add_component_to_this_quaternion(const Quaternion_component& val);
	Quaternion VectorPartOnly(const Quaternion& q) const;
};

class Quaternion_Rotation : public AxisRotation {
public:
	Quaternion_Rotation();
	~Quaternion_Rotation();


	Quaternion GetRotationQuaternion(const Axis_Angle& rotation_about_axis) const;
	Axis_Angle GetRotationAxis(const Quaternion& q) const; /*the input is a quaternion used to defne a rotation in form
		   cos(alpha/2) + \vec{u} * sin(alpha/2)
		   the output is a unit vector which corresponds to the rotation axis
		   and a scalar which is the angle */

	Matrix GetRotationMatrix(const Quaternion& q) const;
	Matrix GetInvRotationMatrix(const Quaternion& q) const { Quaternion q_inv = q; q_inv.Inv(); return GetRotationMatrix(q_inv); }

	Vector Rotate(
		const Axis_Angle& rotation_about_axis,
		const Vector& v
	) const;
	Vector Rotate(
		const Quaternion& q,
		const Vector& v
	) const;

	Quaternion CombineRotation(  /* q = q_2 q_1 */
		const Quaternion& q1/*first rotation quat*/,
		const Quaternion& q2/*second rotation quat*/
	) const;
	Quaternion CombineRotation(
		const Quaternion& q1,
		const Quaternion& q2,
		const Quaternion& q3
	) const;

	Quaternion GetRotation(const Matrix& R) const; /* if R is a rotation matrix then this 
													return the quaternion rotation operator 
													that is equivalent to rotation by this matrix */

protected:
	Quaternion rotationQuaternion(float a, const Quaternion& u) const;
	void Rotate(
		const Vector& u/*unit vector that is the axis of rotation*/,
		float& a/*angle about which vector will be rotated*/,
		Vector& v/*vector that is rotating, reset to new location*/
	) const; /*rotates the vector v about the unit vector u by angle a*/
};
#endif