#pragma once
#include "bsp_part.h"

class bsp_Frame3 : public bsp_part
{
private:
	Vector13 Up;

public:
	Point3d L;
	Point3d R;

	Point3d bigpoint;

	pair<Point3d, Point3d> BorderL;
	pair<Point3d, Point3d> BorderR;

	vector<pair<Point3d, Point3d>> UpCurve;
	vector<pair<Point3d, Point3d>> DownCurve;
	vector<PII> RidgeLine;

	Point3d Pnear2;
	Point3d Pnear3;
	Point3d Pnear4;
	Point3d Pnear5;

public:

	bsp_Frame3();

	void FindLine(Mesh& mesh, Point3d nowNode, Point3d endNode, vector<pair<Point3d, Point3d>> PPSet, vector<Point3d> &RES, int dir);
	void FindLine(Mesh& mesh, Point3d nowNode, Point3d endNode, vector<PII> PIISet, vector<Point3d> &RES, int dir);
	void FindLine(Mesh & mesh, Point3d nowNode, Point3d endNode, set<PII> PIISet, vector<Point3d>& RES, int dir);

	void findfeatureLines(Mesh& mesh);
	void findfeatureVerts(Mesh& mesh);

	void findRidgeLine(Mesh & mesh);

	int Up_Down(Point3d p);
	void initUpDirection();
	void initUpDown(Mesh& mesh);


	~bsp_Frame3();
};
