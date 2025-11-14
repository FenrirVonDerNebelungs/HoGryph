#pragma once
#ifndef AXIS_H
#define AXIS_H

#ifndef MATHBASE_H
#include "../Math/MathBase.h"
#endif
class Axis_Angle : public Vector {
public:
	Axis_Angle();
	Axis_Angle(const Vector& v); /* normalizes vector */
	Axis_Angle(const Vector& u, float angle);
	~Axis_Angle();

	void SetAxis(const Vector& u);/*will normalize vector*/
	void SetAngle(const float& angle);
	void Inv() { m_scalar = -m_scalar; }
	Vector GetAxis() const;
	float GetAngle() const;
protected:
};


class AxisRotation {
public:
	AxisRotation();
	~AxisRotation();

	Axis_Angle GetFlatRotationAxis(
		const Vector& v,
		const Vector& v_new
	) const;

	virtual Vector Rotate(
		const Axis_Angle& rotation_about_axis,
		const Vector& v
	) const;

	Matrix GetRotationMatrix(
		const Vector& new_X,
		const Vector& new_Y,
		const Vector& new_Z) const;/* gets the rotation matrix that rotates the
	X, Y, and Z axis to their new vector positions
	assumes these axis' are orthogonal*/
	void GetAxesFromRotation(
		const Matrix& R,
		Vector& new_X,
		Vector& new_Y,
		Vector& new_Z
	)const;

	virtual Matrix GetRotationMatrix(const Axis_Angle& u_ang) const;
	virtual Matrix GetInvRotationMatrix(const Axis_Angle& u_ang) const {
		Axis_Angle u_ang_inv(u_ang);
		u_ang_inv.Set_Scalar(- u_ang.Get_Scalar());
		return GetRotationMatrix(u_ang_inv);
	}
	Axis_Angle GetRotation(const Matrix& M_Rot) const;/* Unreliable, does not return the correct value for all matrixes. 
	                                                   This method attempts to find the rotation axis by finding the eigenvector of the rotation matrix.
													   The solution to the eigenvector problem is numerically unstable as it relies on exact cancellation.
													   There may also be bugs not caused by rounding errors.
													   */
	float GetRotationAngle(
		const Vector& rotation_axis,
		const Vector& v,
		const Vector& v_new) const;
protected:
};
#endif