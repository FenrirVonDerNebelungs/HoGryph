#pragma once
#ifndef EULER_H
#define EULER_H

#ifndef MATHBASE_H
#include "../Math/MathBase.h"
#endif

class Euler_Rotation {
public:
	Euler_Rotation();
	~Euler_Rotation();

	Matrix GetRx(const float& ang) const;
	Matrix GetRy(const float& ang) const;
	Matrix GetRz(const float& ang) const;

	Matrix GetRotationMatrix(
		const float& phi/*first rotation about z*/, 
		const float& theta/*second rotation, about x*/, 
		const float& psi/*final rotation, about z*/
	) const; /*RzRxRz rotation*/
	Matrix GetInvRotationMatrix(
		const float& phi,
		const float& theta,
		const float& psi
	) const;
	void Rotate(
		const float& phi,
		const float& theta,
		const float& psi,
		Vector& v
	) const;

	void GetRotationAngles(
		const Matrix& R,
		float& phi,
		float& theta,
		float& psi) const;
protected:
};
#endif