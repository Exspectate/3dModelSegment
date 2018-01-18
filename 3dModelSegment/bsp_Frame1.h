#pragma once
#include "bsp_part.h"
class bsp_Frame1 : public bsp_part
{
public:	
	Point3d L;
	Point3d R;
	vector<Point3d> Curve;

    vector<pair<Point3d, Point3d>> UpCurve;
    vector<pair<Point3d, Point3d>> DownCurve;

public:
	bsp_Frame1();
	~bsp_Frame1();

	void findfeatureLines(Mesh& mesh);
	void findfeatureVerts(Mesh& mesh);

	void findLR(Mesh & mesh);
	void findCurve(Mesh& mesh);
	void initUpDown(Mesh& mesh);
};

