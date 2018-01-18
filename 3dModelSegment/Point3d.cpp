#include"Point3d.h"
Point3d::Point3d() { color = 0; };
Point3d::Point3d(double x, double y, double z)
{
	color = 0;
	isBad = false;
	isRidge = false;
	this->x = x;
	this->y = y;
	this->z = z;
}
void Point3d::setIdx(int a) {
	idx = a;
}
bool Point3d::operator== (const Point3d& pt) const
{
	return (sign(x - pt.x) == 0) && (sign(y - pt.y) == 0) && (sign(z - pt.z) == 0);
}
Point3d Point3d::operator + (const Point3d &pt) const
{
	return Point3d(x + pt.x, y + pt.y, z + pt.z);
}
Point3d Point3d::operator - (const Point3d &pt) const
{
	return Point3d(x - pt.x, y - pt.y, z - pt.z);
}
Point3d Point3d::operator * (const double t) const
{
	return Point3d(x*t, y*t, z*t);
}
double Point3d::len()
{
	return sqrt(x*x + y * y + z * z);
}
bool Point3d::operator < (const Point3d &a) const
{
	if (sign(x - a.x) != 0) return x<a.x;
	if (sign(y - a.y) != 0) return y<a.y;
	if (sign(z - a.z) != 0) return z<a.z;
	return 0;
}
