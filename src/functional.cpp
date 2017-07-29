#include <iostream>
#include <math.h>
#include <vector>
#include "eigen/Eigen/Dense"

using Eigen::MatrixXf;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::VectorXf;
using namespace std;

float theta(Vector4f v) {
	float length = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	return (length * 3.14159265 / 180);
}

Matrix3f crossMatrix(Vector4f v) {
	Matrix3f rx(3,3);
	rx(0,0) = 0;
	rx(1,0) = v[2];
	rx(2,0) = -v[1];
	rx(0,1) = -v[2];
	rx(1,1) = 0;
	rx(2,1) = v[0];
	rx(0,2) = v[1];
	rx(1,2) = -v[0];
	rx(2,2) = 0;
	//cout << "crossMatrix: " << endl << rx << endl;
	return rx;
}

Matrix3f rotationMatrix(Vector4f v) {
	Vector3f r(v(0), v(1), v(2));
	float rnorm = r.norm();
	Matrix3f I = Matrix3f::Identity(3,3);
	if(rnorm == 0){
		return I;
	}
	float t = theta(v);
	if(rnorm != 0){
		v[0] /= rnorm;
		v[1] /= rnorm;
		v[2] /= rnorm;
		r /= rnorm;
	}
	/*cout << "v: " << endl << v << endl;
	cout << "r: " << endl << r << endl;
	cout << "r.transpose(): " << endl << r.transpose();*/
	Matrix3f rx = crossMatrix(v);
	return r*r.transpose() + sin(t)*rx - cos(t)*rx*rx;
}

Matrix4f xValMatrix(Vector4f v, MatrixXf Rot) {
	Matrix4f m = MatrixXf::Identity(4,4);
	m(0,0) = Rot(0,0);
	m(0,1) = Rot(0,1);
	m(0,2) = Rot(0,2);
	m(0,3) = v(0);
	m(1,0) = Rot(1,0);
	m(1,1) = Rot(1,1);
	m(1,2) = Rot(1,2);
	m(1,3) = v(1);
	m(2,0) = Rot(2,0);
	m(2,1) = Rot(2,1);
	m(2,2) = Rot(2,2);
	m(2,3) = v(2);
	m(3,0) = 0;
	m(3,1) = 0;
	m(3,2) = 0;
	m(3,3) = 1;
	return m;
}

MatrixXf Jacobian(std::vector<Vector4f*> points, std::vector<Matrix3f> rotations, int n) {

	std::vector<Vector4f> Pi;
	std::vector<Matrix3f> Ri_to_0;
	std::vector<Matrix4f> Xi;
	std::vector<Matrix4f> Xn_to_i;
	std::vector<Matrix3f> J;

	for (int i = 0; i < n; i++) {
		Matrix3f rotation = rotations[i];
		Vector4f point = *(points[i]);
		Vector3f p_i(point(0), point(1), point(2));
		p_i = rotation * p_i;
		Pi.push_back(Vector4f (p_i(0), p_i(1), p_i(2), 1));
	}


	for (int i = 0; i < n; i++) {

		Matrix3f rotation = rotations[i];

		if (i > 0) {
			Ri_to_0.push_back(Ri_to_0[i-1]*rotation);
		} else {
			Ri_to_0.push_back(rotation);
		}
		
		Matrix4f m = xValMatrix(Pi[i], rotation);
		Xi.push_back(m);
	}

	for (int i = 0; i < n; i++) {
		int k = i+1;
		Matrix4f Xn = MatrixXf::Identity(4,4);
		if (i == n-1) {
 			Xn_to_i.push_back(Xn);
		} else {
			while (k < n) {
				Xn *= Xi[k];
				k += 1;
			}
			Xn_to_i.push_back(Xn);
		}
	}

	for (int i = 0; i < n; i++) {
		Matrix3f m = crossMatrix(Xn_to_i[i]*Pi[i]);
		J.push_back(-1*Ri_to_0[i]*m);
	}

	MatrixXf jacobian(3, 0);
	for (int i = 0; i < n; i++) {
		jacobian.conservativeResize(3, jacobian.cols()+3);
		jacobian.col(jacobian.cols()-3) = J[i].col(0);
		jacobian.col(jacobian.cols()-2) = J[i].col(1);
		jacobian.col(jacobian.cols()-1) = J[i].col(2);
	}
	return jacobian;

}

MatrixXf JacobianInverse(MatrixXf j, float threshold, int n) {
	Eigen::JacobiSVD<MatrixXf> svd(j, Eigen::ComputeFullU | Eigen::ComputeFullV);
	MatrixXf U = svd.matrixU();
	MatrixXf V = svd.matrixV();
	MatrixXf sv = svd.singularValues();

	for (int i = 0; i < 3; i++) {
		if (sv(i) < threshold) {
			sv(i) = 0;
		} else {
			sv(i) = 1 / sv(i);
		}
	}

	MatrixXf D = MatrixXf::Zero(3*n, 3);
	D(0,0) = sv(0);
	D(1,1) = sv(1);
	D(2,2) = sv(2);

	return V*D*U.transpose();
}

std::vector<Vector4f*> absolute(std::vector<Vector4f*> points, int n){
	std::vector<Vector4f*> to_return;
	Vector4f to_add;
	to_return.push_back(points[0]);
	for(int i = 1; i < n; i++){
		//cout << "absolute, i: " << i << endl;
		to_add = *(new Vector4f(points[i][0][0], points[i][0][1], points[i][0][2], 0));
		//cout << "to_add: " << endl << to_add << endl;
		//cout << "*to_return[i-1]: " << endl << *to_return[i-1] << endl;
		to_add += *to_return[i-1];
		Vector4f* to_push = new Vector4f(to_add[0], to_add[1], to_add[2], 1);
		//cout << "to_add+=: " << endl << to_add << endl;
		to_return.push_back(to_push);
	}
	//cout << "finalized!" << endl;
	/*for(int i = 0; i < n; i++){
		cout << "i: " << i << endl;
		cout << *to_return[i] << endl;
	}*/
	return to_return;
}

float normalize(Vector4f vec){
	return sqrt(vec(0)*vec(0) + vec(1)*vec(1) + vec(2)*vec(2));
}

std::vector<Vector4f*> moveToGoal(Vector4f goal, std::vector<Vector4f*> points, float error, int n) {

	std::vector<Matrix3f> rotations;
	std::vector<Vector4f*> newPoints;
	std::vector<Vector4f*> startPoints;
	std::vector<Vector4f*> abs_points = absolute(points, n);

	for(int i = 0; i < n; i++){
		startPoints.push_back(points[i]);
	}
	// STEP 1
	rotations.push_back(rotationMatrix(Vector4f (0, 0, 0, 1)));
	rotations.push_back(rotationMatrix(Vector4f (0, 0, 0, 1)));
	rotations.push_back(rotationMatrix(Vector4f (0, 0, 0, 1)));
	rotations.push_back(rotationMatrix(Vector4f (0, 0, 0, 1)));

	float step = 5 *(3.14159265/180);
	// STEP 2
	while ((*(abs_points[n-1]) - goal).norm() > error) {
		//cout << (goal - *(abs_points[n-1])).norm() << endl;
		// STEP 3
		MatrixXf j = Jacobian(startPoints, rotations, n);
		//cout << "j: " << endl << j << endl;
		MatrixXf jInv = JacobianInverse(j, .01, n);
		//cout << "jInv: " << endl << jInv << endl;
		// STEP 4
		Vector4f pToG = step * (goal - *(abs_points[n-1]));
		//cout << "step: " << step << endl;
		//cout << "pToG: " << endl << pToG << endl;
		//cout << "pToG.norm(): " << pToG.norm() << endl;
		//cout << "error_comparison: " << (goal - *(abs_points[n-1])).norm() << endl;
		//cout << "error: " << error << endl;
		Vector3f pToG_temp (pToG(0), pToG(1), pToG(2));
		VectorXf dr = jInv*pToG_temp;
		//cout << "dr: " << endl << dr << endl;
		// STEP 5
		newPoints.clear();
		for (int i = 0; i < n; i++) {
			//cout << "i: " << i << endl;
			rotations[i] *= rotationMatrix(Vector4f (dr(3*i), dr(3*i+1), dr(3*i+2), 0));
			//cout << "to_add2: " << endl << *rotations[i] << endl;
			Vector4f point = *(startPoints[i]);
			//cout << "to_add3: " << endl << *rotations[i] << endl;
			Vector3f tempPoint (point[0], point[1], point[2]);
			//cout << "to_add4: " << endl << *rotations[i] << endl;
			Vector3f tempNewPoint = rotations[i] * tempPoint;
			//cout << "rotations[i]: " << endl << rotationMatrix(*(rotations[i])) << endl;
			//cout << "newPoint: " << endl << tempNewPoint << endl;
			newPoints.push_back(new Vector4f(tempNewPoint(0), tempNewPoint(1), tempNewPoint(2), 1));
		}

		Vector4f newEndEffector = *(absolute(newPoints, n)[n-1]);
		Vector4f oldEndEffector = *(abs_points[n-1]);
		//cout << "newEndEffector: " << endl << newEndEffector << endl;
		//std::cout << (newEndEffector-goal).norm() << std::endl;
		//std::cout << (oldEndEffector-goal).norm() << std::endl;

		if (((newEndEffector-goal).norm() > (oldEndEffector-goal).norm()) && step > .05) {
			//std::cout << "step: " << step << std::endl;
			step /= 2;
			// }
			//std::cout << "Step size too big" << std::endl;
			//cout << "";
		} else {
			points.clear();
			for(int i = 0; i < n; i++){
				points.push_back(newPoints[i]);
			}

			//std::cout << "Moving closer!" << std::endl;

			//std::cout << *(points[n-1]) << std::endl;
			//std::cout << newEndEffector << std::endl;
			//RENDER
		}
		abs_points = absolute(points, n);
		/*cout << normalize(*abs_points[0]) << endl;
		for(int i = 1; i < n; i++){
			cout << normalize((*abs_points[i]-(*abs_points[i-1]))) << endl;
		}*/
		//cout << "abs: " << endl << *abs_points[0] << endl << *abs_points[1] << endl << *abs_points[2] << endl << *abs_points[3] << endl;
	}
	return points;
}

int main()
{
	std::vector<Vector4f*> points;
	int n = 4;
	float error = .001f;

	Vector4f test(0,0,10,0);
	//cout << "test: " << endl << rotationMatrix(test) << endl;

	points.push_back(new Vector4f (1, 0, 0, 1));
	points.push_back(new Vector4f (1, 0, 0, 1));
	points.push_back(new Vector4f (1, 0, 0, 1));
	points.push_back(new Vector4f (1, 0, 0, 1));


	float r = 4.0;
	float th = 1 *(3.14159265/180);
	Vector4f goal(r*cos(th), r*sin(th), 0, 1.0);

	while(true) {

		points = moveToGoal(goal, points, .02, n);
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		cout << "MADE IT" << endl;
		th += 1 *(3.14159265/180);
		goal(0) = r*cos(th);
		goal(1) = r*sin(th);
	}

	return 0;
}
