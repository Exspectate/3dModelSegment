#ifndef _3dOperator_H_
#define _3dOperator_H_
#include "Point3d.h"
#include <stdio.h>
double dot(const Point3d &a,const Point3d &b);
Point3d det(const Point3d &a,const Point3d &b);
Point3d pVec(const Point3d &a,const Point3d &b,const Point3d &c);
bool lineInPlane(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1);
bool VecParaVec(const Point3d &a,const Point3d &b);
bool lineParaLine(const Point3d &a0,const Point3d &b0,const Point3d &a1,const Point3d &b1);
bool lineParaPlane(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1);
Point3d intersection(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1);
Point3d intersection(const Point3d &a0,const Point3d &b0,const Point3d &a1,const Point3d &b1);
bool pointInTri(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &p);
bool rayByPoint(const Point3d &l0,const Point3d &l1,const Point3d &p);
bool rayByTri(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1);
bool segByPlane(const Point3d &a,const Point3d &b,const Point3d &c,const Point3d &l0,const Point3d &l1);
void show(const Point3d &a);
void readPt(Point3d &a);
bool PlaneByTri(const Point3d & a, const Point3d & b, const Point3d & c, const Point3d & ta, const Point3d & tb, const Point3d & tc, Point3d & l0, Point3d & l1);
#endif