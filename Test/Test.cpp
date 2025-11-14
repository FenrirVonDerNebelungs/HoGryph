#include "Test.h"

Test::Test() {
	;
}
Test::~Test() {
	;
}

void Test::RunTests() {
	AxisRotation Axis_rotor;
	Quaternion_Rotation Quat_rotor;
	std::cout << "\n Rotations about a vector \n";
	float angle = GetAngle();
	std::cout << "\n  Rotation axis ";
	Vector v_axis = GetVector();
	Axis_Angle rot_axis_ang(v_axis);
	rot_axis_ang.SetAngle(angle);
	std::cout << "\n  Rotations will be around axis, angle ";
	ShowAxisAngle(rot_axis_ang);

	std::cout << "\n\n --------------------- \n";
	std::cout << "\n Rotate a test vector \n";
	std::cout << "\n  Set test ";
	Vector v_test_vector = GetVector();
	Vector rotated_vector1 = Axis_rotor.Rotate(rot_axis_ang, v_test_vector);
	Vector rotated_vector2 = Quat_rotor.Rotate(rot_axis_ang, v_test_vector);
	std::cout << "\n  Rotated vector about axis: ";
	ShowVector(rotated_vector1);
	std::cout << "\n  Rotated vector by quaternion: ";
	ShowVector(rotated_vector2);
	std::cout << "\n\n --------------------- \n";
	std::cout << " Test of Axis_Angle <-> Quaternion conversion\n";
	std::cout << "    Quaternion: \n";
	Quaternion rot_quat_from_rot_axis_ang = Quat_rotor.GetRotationQuaternion(rot_axis_ang);
	ShowQuaternion(rot_quat_from_rot_axis_ang);
	Axis_Angle rot_axis_ang_from_quat = Quat_rotor.GetRotationAxis(rot_quat_from_rot_axis_ang);
	std::cout << "   Test axis angle: \n";
	ShowAxisAngle(rot_axis_ang_from_quat);
	std::cout << "      should match: \n";
	ShowAxisAngle(rot_axis_ang);
	std::cout << "\n\n --------------------- \n";
	std::cout << "Matrix from rotation about axis by angle: ";
	Matrix rotated_matrix1 = Axis_rotor.GetRotationMatrix(rot_axis_ang);
	Quaternion rot_quat = Quat_rotor.GetRotationQuaternion(rot_axis_ang);
	Matrix rotated_matrix2 = Quat_rotor.GetRotationMatrix(rot_quat);
	std::cout << "\n  rotation matrix: "; 
	ShowMatrix(rotated_matrix1);
	std::cout << "\n  rotation matrix from quat: ";
	ShowMatrix(rotated_matrix2);
	std::cout << "\n\n --------------------- \n";
	std::cout << " tests of rotation angle of axis.h\n";
	Vector X_Unit_axis(1.f, 0.f, 0.f);
	if (v_axis.Dot(X_Unit_axis) < 0.9f) {
		std::cout << "rotate X axis:\n" << "   start with: \n";
		ShowVector(X_Unit_axis);
		Vector X_unit_axis_rotated = Axis_rotor.Rotate(rot_axis_ang, X_Unit_axis);
		std::cout << "    rotates to: \n";
		ShowVector(X_unit_axis_rotated);
		std::cout << "    rotated about angle: " << rot_axis_ang.GetAngle() << "\n";
		float test_rotation_angle = Axis_rotor.GetRotationAngle(v_axis, X_Unit_axis, X_unit_axis_rotated);
		std::cout << "    rotated angle from GetRotationAngle: " << test_rotation_angle << "\n";
	}
	std::cout << "\n\n --------------------- \n";
	std::cout << " inv tests of Axis.h \n";
	std::cout << "  inv rotation matrix: ";
	Matrix inv_rotated_matrix1 = Axis_rotor.GetInvRotationMatrix(rot_axis_ang);
	ShowMatrix(inv_rotated_matrix1);
	std::cout << " matrix inv test: ";
	Matrix inv_test1 = inv_rotated_matrix1 * rotated_matrix1;
	ShowMatrix(inv_test1);
	Axis_Angle test_rot_axis_1 = Axis_rotor.GetRotation(rotated_matrix1);
	std::cout << "  test of matrix -> axis,angle should be: \n";
	Axis_Angle norm_rot_axis_ang(rot_axis_ang);
	norm_rot_axis_ang.Normalize();
	ShowAxisAngle(norm_rot_axis_ang);
	std::cout << "\n   rotation axis and angle are: ";
	ShowAxisAngle(test_rot_axis_1);
	std::cout << "\n\n --------------------- \n";
	std::cout << " inv test of Quaternion\n";
	std::cout << "   inv quat rotation matrix: ";
	Matrix inv_rotated_matrix2 = Quat_rotor.GetInvRotationMatrix(rot_quat);
	ShowMatrix(inv_rotated_matrix2);
	std::cout << "   quat matrix inv test: ";
	Matrix inv_test2 = inv_rotated_matrix2 * rotated_matrix2;
	ShowMatrix(inv_test2);
	std::cout << "\n   test quaternion to rotation about axis ";
	std::cout << "\n      axis rot from quaternion should equal:\n";
	ShowAxisAngle(rot_axis_ang);
	std::cout << "\n      axis rot is:\n";
	Axis_Angle reversed_rot_quat = Quat_rotor.GetRotationAxis(rot_quat);
	ShowAxisAngle(reversed_rot_quat);
	std::cout << "\n   test matrix to quaternion conversion ";
	std::cout << "\n      quaternion from matrix: \n       ";
	Quaternion quat_from_rotated_matrix2 = Quat_rotor.GetRotation(rotated_matrix2);
	ShowQuaternion(quat_from_rotated_matrix2);
	std::cout << "\n      matrix from quaternion: \n";
	Matrix mat_from_quat_from_rot_mat2 = Quat_rotor.GetRotationMatrix(quat_from_rotated_matrix2);
	ShowMatrix(mat_from_quat_from_rot_mat2);
	std::cout << "\n       above should equal: \n";
	ShowMatrix(rotated_matrix2);
	std::cout << "\n\n --------------------- \n";
	std::cout << " tests of double rotation\n";
	std::cout << "\n   Get a second rotaton axis \n";
	float angle_ = GetAngle();
	Vector v_axis_ = GetVector();
	Axis_Angle rot_axis_ang_(v_axis_);
	rot_axis_ang_.Set_Scalar(angle_);
	std::cout << "\n   double quaternion rotation \n";
	/* rot_quat is quaternion for 1st rotation, corresponds to rotated_matrix2 */
	Quaternion rot_quat_ = Quat_rotor.GetRotationQuaternion(rot_axis_ang_);
	std::cout << "\n       1st quat rotation: ";
	ShowQuaternion(rot_quat);
	std::cout << "\n       2nd quat rotation: ";
	ShowQuaternion(rot_quat_);
	Quaternion combined_rot_quat = Quat_rotor.CombineRotation(rot_quat, rot_quat_);
	std::cout << "\n         combined rotation: ";
	ShowQuaternion(combined_rot_quat);
	std::cout << "\n         combined rotation as a matrix: ";
	Matrix combined_rot_quat_M = Quat_rotor.GetRotationMatrix(combined_rot_quat);
	ShowMatrix(combined_rot_quat_M);
	std::cout << "\n    double rotation using Axis.h: ";
	std::cout << "\n       1st matrix for rotation: ";
	ShowMatrix(rotated_matrix1);
	std::cout << "\n       2nd matrix for rotation: ";
	Matrix rotated_matrix1_ = Axis_rotor.GetRotationMatrix(rot_axis_ang_);
	ShowMatrix(rotated_matrix1_);
	std::cout << "\n       combined rotation as a matrix: ";
	Matrix combined_matrix_rotation_M = rotated_matrix1_ * rotated_matrix1;
	ShowMatrix(combined_matrix_rotation_M);
	std::cout << "\n    axis for double rotation using quaternions";
	RotationTrans RotTrans_rotor;
	Axis_Angle combined_single_axis_quat_method = RotTrans_rotor.CombineRotation(rot_axis_ang, rot_axis_ang_);
	std::cout << "\n        combined axis angle using quat method: ";
	ShowAxisAngle(combined_single_axis_quat_method);
	std::cout << "\n        combined axis using matrix eigenvector: ";
	Axis_Angle combined_single_axis_eigen_method = Axis_rotor.GetRotation(combined_matrix_rotation_M);
	ShowAxisAngle(combined_single_axis_eigen_method);
	TestEuler(rotated_matrix1);
	/* 0 1 0, 208 degrees
	*  0.1 2 0.3 20 degrees
	*/
}
void Test::TestEuler(const Matrix& rotated_matrix1) {
	Euler_Rotation Euler_rotor;
	std::cout << "\n\n --------------------- \n";
	std::cout << " tests of Euler rotation\n";
	std::cout << "\n   Start with matrix for rotation: \n";
	ShowMatrix(rotated_matrix1);
	std::cout << "\n   Find Euler angles from matrix: \n";
	float phi, theta, psi;
	Euler_rotor.GetRotationAngles(rotated_matrix1, phi, theta, psi);
	ShowEulerAngles(phi, theta, psi);
	std::cout << "\n   Find matrix from Euler angles should return previous matrix: \n";
	Matrix test_rotated_matrix1 = Euler_rotor.GetRotationMatrix(phi, theta, psi);
	ShowMatrix(test_rotated_matrix1);
	std::cout << "\n   Find matrix from matrix multiplication for Euler angles: \n";
	Matrix test_3matrix_rotated_matrix1 = TestEulerFrom3MatrixRot(phi, theta, psi);
	ShowMatrix(test_3matrix_rotated_matrix1);
	std::cout << "\n   Find inv matrix from Euler angles: \n";
	Matrix test_inv_rotated_matrix1 = Euler_rotor.GetInvRotationMatrix(phi, theta, psi);
	ShowMatrix(test_inv_rotated_matrix1);
	std::cout << "\n       test inv, should return I: \n";
	Matrix test_inv = test_inv_rotated_matrix1 * test_rotated_matrix1;
	ShowMatrix(test_inv);
}
Matrix Test::TestEulerFrom3MatrixRot(const float& phi, const float& theta, const float& psi) {
	Euler_Rotation Euler_rotor;
	Matrix Rz_phi = Euler_rotor.GetRz(phi);
	Matrix Rx_theta = Euler_rotor.GetRx(theta);
	Matrix Rz_psi = Euler_rotor.GetRz(psi);
	Matrix Rot = Rz_psi * Rx_theta * Rz_phi;
	return Rot;
}
float Test::GetAngle() {
	float angle;
	std::cout << " angle (degrees): ";
	std::cin >> angle;
	angle *= (PI_f / 180.f);
	return angle;
}
Vector Test::GetVector(){
	float x, y, z;
	std::cout << "\n vector \n  x: ";
	std::cin >> x;
	std::cout << "  y: ";
	std::cin >> y;
	std::cout << "  z: ";
	std::cin >> z;
	std::cout << "\n\n";
	Vector c_in_vec(x, y, z);
	return c_in_vec;
}
void Test::ShowVector(const Vector& vec) {
	std::cout << " x:" << vec.Get_i(0) << ", y:" << vec.Get_i(1) << ", z:" << vec.Get_i(2) << "\n";
}
void Test::ShowMatrix(const Matrix& M) {
	printf("\n | %2.6f  %2.6f  %2.6f |\n", M.Get_ij(0, 0), M.Get_ij(0, 1), M.Get_ij(0, 2));
	printf(" | %2.6f  %2.6f  %2.6f |\n", M.Get_ij(1, 0), M.Get_ij(1, 1), M.Get_ij(1, 2));
	printf(" | %2.6f  %2.6f  %2.6f |\n\n", M.Get_ij(2, 0), M.Get_ij(2, 1), M.Get_ij(2, 2));
}
void Test::ShowAxisAngle(const Axis_Angle& Ax) {
	std::cout << " angle: " << Ax.Get_Scalar();
	std::cout << "\n";
	ShowVector(Ax);
	std::cout << "\n";
}
void Test::ShowQuaternion(const Quaternion& q) {
	printf(" %2.2f + %2.2fi + %2.2fj + %2.2fk ", q.Get_Scalar(), q.Get_i(0), q.Get_i(1), q.Get_i(2));
}
void Test::ShowMatrixTerm(const Matrix& M, const int& i, const int& j) {
	printf(" Term i: %1u,   j: %1u is %2.2f ", i, j, M.Get_ij(i, j));
}
void Test::ShowEulerAngles(const float& phi, const float& theta, const float& psi) {
	float rad_to_deg = 180.f / PI_f;
	float d_phi = phi * rad_to_deg;
	float d_theta = theta * rad_to_deg;
	float d_psi = psi * rad_to_deg;
	printf(" phi: %3f, theta: %3f, psi: %3f ", d_phi, d_theta, d_psi);
}