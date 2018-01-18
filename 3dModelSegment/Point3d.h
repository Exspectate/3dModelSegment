#ifndef _Point3d_H_
#define _Point3d_H_
#include "Util.h"
#include <math.h>
class Point3d {
public:
	double x, y, z;//坐标
	double Cf;
	int color;
	int idx;//索引
	bool isRidge;		//是否棱边上点
	bool isBad;

public:
	Point3d();
	Point3d(double x, double y, double z);
	void setIdx(int a);
	bool operator== (const Point3d& pt) const;
	Point3d operator + (const Point3d &pt) const;
	Point3d operator - (const Point3d &pt) const;
	Point3d operator * (const double t) const;
	double len();
	
	bool operator < (const Point3d &a) const;
};
#endif