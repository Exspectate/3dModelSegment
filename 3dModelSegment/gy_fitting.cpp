#include "gy_fitting.h"
#include <math.h>
#include <iostream>


using namespace std;
using namespace Eigen;

static Matrix<complex<double>, Dynamic, Dynamic> D;

static bool cmp(int a, int b) {  
	return D(a).real() > D(b).real();
}

//对给定点集合拟合一个平面。points为待拟合的点，plane为拟合的平面(三点表示)
void fittingPlane(vector<Point3d> &points, vector<Point3d> &plane) {
	size_t n = points.size();
	MatrixXd X(3, n);
	MatrixXd mu = MatrixXd::Zero(3, 1);
	for (int j = 0; j < n; ++j) {
		X(0, j) = points[j].x;
		X(1, j) = points[j].y;
		X(2, j) = points[j].z;
		mu = mu + X.col(j);
	}
	mu = mu / n;
	for (int j = 0; j < n; ++j) {
		X.col(j) = X.col(j) - mu;
	}
	MatrixXd C(X*X.transpose());
	EigenSolver<MatrixXd> eig(C);
	int ind[3] = { 0, 1, 2 };
	D = eig.eigenvalues();
	Matrix<complex<double>, Dynamic, Dynamic> P(eig.eigenvectors());
	//cout << D.size() << "\n";
	//for (int i = 0; i < 3; ++i) printf("(%.3f, %.3f)  ", D(ind[i]).real(), D(ind[i]).imag());
	//printf("\n");
	sort(ind, ind + 3, cmp);  //特征值从大到小下标排序
	//for (int i = 0; i < 3; ++i) printf("(%.3f, %.3f)  ", D(ind[i]).real(), D(ind[i]).imag());
	//printf("\n");
	MatrixXd Q(3, 2);
	Q.col(0) = P.col(ind[0]).real();
	//for (int i = 0; i < 3; ++i) printf("%.3f  ", Q.col(0)(i));
	//printf("\n");
	Q.col(1) = P.col(ind[1]).real();
	Q.col(1) = Q.col(1)- (Q.col(1).transpose()*Q.col(0))*Q.col(0);//正交化
	Q.col(1) = Q.col(1) / sqrt(Q.col(1).transpose()*Q.col(1)); //单位化
	//for (int i = 0; i < 3; ++i) printf("%.3f  ", Q.col(1)(i));
	//printf("\n");
	plane.resize(3);
	plane[0].x = mu(0);
	plane[0].y = mu(1);
	plane[0].z = mu(2);
	Q.col(0) = Q.col(0) + mu;
	plane[1].x = Q(0, 0);
	plane[1].y = Q(1, 0);
	plane[1].z = Q(2, 0);
	Q.col(1) = Q.col(1) + mu;
	plane[2].x = Q(0, 1);
	plane[2].y = Q(1, 1);
	plane[2].z = Q(2, 1);
}

//对给定点集合拟合一条直线。points为待拟合的点，line为拟合的直线(两点表示)
void fittingLine(vector<Point3d> &points, vector<Point3d> &line) {
	int n = points.size();
	MatrixXd X(3, n);
	MatrixXd mu = MatrixXd::Zero(3, 1);
	for (int j = 0; j < n; ++j) {
		X(0, j) = points[j].x;
		X(1, j) = points[j].y;
		X(2, j) = points[j].z;
		mu = mu + X.col(j);
	}
	mu = mu / n;
	for (int j = 0; j < n; ++j) {
		X.col(j) = X.col(j) - mu;
	}
	MatrixXd C(X*X.transpose());
	EigenSolver<MatrixXd> eig(C);
	D = eig.eigenvalues();
	Matrix<complex<double>, Dynamic, Dynamic> P(eig.eigenvectors());

	int ind[3] = { 0, 1, 2 };
	sort(ind, ind + 3, cmp);  //特征值从大到小下标排序

	MatrixXd Q(3, 2);
	Q.col(0) = P.col(ind[0]).real();
	line.resize(2);
	line[0].x = mu(0);
	line[0].y = mu(1);
	line[0].z = mu(2);
	Q.col(0) = Q.col(0) + mu;
	line[1].x = Q(0, 0);
	line[1].y = Q(1, 0);
	line[1].z = Q(2, 0);
}


//test code
//int main() {
//	vector<Point3d> points(4), plane;
//	points[0].x = 0; points[0].y = 0; points[0].z = 0;
//	points[1].x = 1; points[1].y = 0; points[1].z = 0;
//	points[2].x = 0; points[2].y = 1; points[2].z = 0;
//	points[3].x = 1; points[3].y = 1; points[3].z = 0;
//	fittingPlane(points, plane);
//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
//	for (int i = 0; i < 3; ++i)
//		printf("(%.3f, %.3f, %.3f)   ", plane[i].x, plane[i].y, plane[i].z);
//	printf("\n\n");
//
//	points[0].x = 1; points[0].y = 0; points[0].z = 0;
//	points[1].x = 0; points[1].y = 1; points[1].z = 0;
//	points[2].x = 0; points[2].y = 0; points[2].z = 1;
//	points[3].x = 0.5; points[3].y = 0.5; points[3].z = 0;
//	fittingPlane(points, plane);
//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
//	for (int i = 0; i < 3; ++i)
//		printf("(%.3f, %.3f, %.3f)   ", plane[i].x, plane[i].y, plane[i].z);
//	return 0;
//}