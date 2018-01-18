#include "3dOperator.h"
double dot(const Point3d &a,const Point3d &b)
{
	return a.x*b.x+a.y*b.y+a.z*b.z;
}

Point3d det(const Point3d &a,const Point3d &b)
{
	return Point3d(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);
}

Point3d pVec(const Point3d &a,const Point3d &b,const Point3d &c)
{
	return det(a-b,b-c);
}

bool lineInPlane(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1)
{
	Point3d p = pVec(a,b,c);
	if(sign(dot(p,l1-l0))==0)
	{
		if(a==l0)
			if(sign(dot(p,b-l0))==0)
				return true;
		if(sign(dot(p,a-l0))==0)
			return true;
	}
	return false;
}

bool VecParaVec(const Point3d &a,const Point3d &b)
{
	if(sign(a.x/b.x-a.y/b.y)==0&&sign(a.x/b.x-a.z/b.z)==0)
		return true;
	return false;
}

bool lineParaLine(const Point3d &a0,const Point3d &b0,const Point3d &a1,const Point3d &b1)
{
	Point3d l1 = a0-b0;
	Point3d l2 = a1-b1;
	return VecParaVec(l1,l2);
}

bool lineParaPlane(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1)
{
	Point3d p = pVec(a,b,c);
	if(sign(dot(p,l1-l0))!=0)
		return false;
	if(a==l0)
		if(sign(dot(p,b-l0))==0)
			return false;
	else if(sign(dot(p,a-l0))==0)
		return false;
	return true;
}

Point3d intersection(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1)
{
	Point3d p = pVec(a,b,c);
	double t = (p.x * (a.x - l0.x) + p.y * (a.y - l0.y) + p.z * (a.z - l0.z))/(p.x * (l1.x - l0.x) + p.y * (l1.y - l0.y) + p.z * (l1.z - l0.z));
	return l0 + ((l1 - l0)*t);
}

Point3d intersection(const Point3d &a0,const Point3d &b0,const Point3d &a1,const Point3d &b1)
{
	double t = ((a0.x - a1.x) * (a1.y - b1.y) - (a0.y - a1.y) * (a1.x - b1.x))/((a0.x - b0.x) * (a1.y - b1.y) - (a0.y - b0.y) * (a1.x - b1.x));
	return a0 + ((b0-a0)*t);
}

bool pointInTri(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &p)
{
	return sign(det(a-b,a-c).len() - det(p-a,p-b).len() - det(p-b,p-c).len() - det(p-c,p-a).len()) == 0;
}

bool rayByPoint(const Point3d &l0,const Point3d &l1,const Point3d &p)
{
	Point3d t1 = p-l0;
	Point3d t2 = l1-l0;
	Point3d t3 = p-l1;
	if(sign(t1.len()-t2.len()-t3.len())==0)
		return true;
	else if(sign(t2.len()-t1.len()-t3.len())==0)
		return true;
	else
		return false;
}

bool rayByTri(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1)
{
	if(lineInPlane(a,b,c,l0,l1))
	{
		if(!lineParaLine(a,b,l0,l1))
		{
			Point3d t = intersection(a,b,l0,l1);
			if(rayByPoint(l0,l1,t)&&pointInTri(a,b,c,t))
				return true;
		}
		if(!lineParaLine(b,c,l0,l1))
		{
			Point3d t = intersection(b,c,l0,l1);
			if(rayByPoint(l0,l1,t)&&pointInTri(a,b,c,t))
				return true;
		}
		if(!lineParaLine(c,a,l0,l1))
		{
			Point3d t = intersection(c,a,l0,l1);
			if(rayByPoint(l0,l1,t)&&pointInTri(a,b,c,t))
				return true;
		}
		return false;
	}
	else
	{
		if(lineParaPlane(a,b,c,l0,l1))
			return false;
		Point3d p = intersection(a,b,c,l0,l1);
		if(pointInTri(a,b,c,p)&&rayByPoint(l0,l1,p)&&!(p==l0))
			return true;
		return false;
	}
	return false;
}

bool segByPlane(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1)
{
	if(lineParaPlane(a,b,c,l0,l1))
		return false;
	Point3d tmp = intersection(a,b,c,l0,l1);
	Point3d t1 = tmp-l0;
	Point3d t2 = tmp-l1;
	Point3d t3 = l1-l0;
	if(sign(t1.len()+t2.len()-t3.len())==0)
		return true;
	return false;
}

void show(const Point3d &a)
{
	printf("(%f,%f,%f)",a.x,a.y,a.z);
}

void readPt(Point3d &a)
{
	scanf_s("%lf%lf%lf",&a.x,&a.y,&a.z);
}
// 平面3点  三角面片3点  生成两个交点
bool PlaneByTri(const Point3d &a, const Point3d &b, const Point3d &c, const Point3d &ta, const Point3d &tb, const Point3d &tc, Point3d &l0, Point3d &l1)
{
	int now = 0;
	if (segByPlane(a, b, c, ta, tb))
	{
		l0 = intersection(a, b, c, ta, tb);
		now++;
	}
	if (segByPlane(a, b, c, tb, tc))
	{
		if (now == 0)
			l0 = intersection(a, b, c, tb, tc);
		else
			l1 = intersection(a, b, c, tb, tc);
		now++;
	}
	if (segByPlane(a, b, c, tc, ta))
	{
		if (now == 0)
			l0 = intersection(a, b, c, tc, ta);
		else
			l1 = intersection(a, b, c, tc, ta);
		now++;
	}
	if (l0 == l1)
		return false;
	if (now == 2)
		return true;
	return false;
}