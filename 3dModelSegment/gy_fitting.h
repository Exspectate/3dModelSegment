#pragma once
#include <vector>
#include <Eigen\Eigen>
#include "Point3d.h"

using namespace std;

void fittingPlane(vector<Point3d> &points, vector<Point3d> &plane); //对给定点集合拟合一个平面。points为待拟合的点，plane为拟合的平面(三点表示)

void fittingLine(vector<Point3d> &points, vector<Point3d> &line); //对给定点集合拟合一条直线。points为待拟合的点，line为拟合的直线(两点表示)
