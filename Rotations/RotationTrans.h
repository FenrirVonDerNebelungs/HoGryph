#pragma once
#ifndef ROTATIONTRANS_H
#define ROTATIONTRANS_H

#ifndef EULER_H
#include "Euler.h"
#endif

#ifndef QUATERNION_H
#include "Quaternion.h"
#endif

/* Convert Euler rotations to quaternion rotations and quaternion rotations to Euler rotations */
class RotationTrans {
public:
	RotationTrans();
	~RotationTrans();

	Quaternion Quaternion_From_Euler(
		const float& phi,
		const float& theta,
		const float& psi
	) const;
	void Euler_From_Quaternion(
		const Quaternion& q,
		float& phi,
		float& theta,
		float& psi
	) const;
	Axis_Angle CombineRotation(
		const Axis_Angle& u_rot_1,
		const Axis_Angle& u_rot_2
	) const {
		return CombineRotation_UsingQuaternions(u_rot_1, u_rot_2);
	}

protected:
	Axis_Angle CombineRotation_UsingQuaternions(
		const Axis_Angle& u_rot_1,
		const Axis_Angle& u_rot_2
	) const;

};

#endif
