#pragma once
#ifndef TEST_H
#define TEST_H
#include <iostream>
#include <istream>
#include <cstdio>

#ifndef ROTATIONTRANS_H
#include "../Rotations/RotationTrans.h"
#endif

class Test {
public:
	Test();
	~Test();

	void RunTests();

protected:
	friend Matrix;

	void ShowEulerAngles(const float& phi, const float& theta, const float& psi);
	Matrix TestEulerFrom3MatrixRot(const float& phi, const float& theta, const float& psi);
	Vector GetVector();
	float GetAngle();
	void ShowVector(const Vector& vec);
	void ShowMatrix(const Matrix& M);
	void ShowAxisAngle(const Axis_Angle& Ax);
	void ShowQuaternion(const Quaternion& q);
	void ShowMatrixTerm(const Matrix& M, const int& i, const int& j);

	void TestEuler(const Matrix& rotated_matrix1);
};
#endif TEST_H

