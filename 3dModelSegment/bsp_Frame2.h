#pragma once
#include "bsp_part.h"



class bsp_Frame2 : public bsp_part
{
public:
	Point3d L;
	Point3d R;

    vector<pair<Point3d, Point3d>> UpCurve;
    vector<pair<Point3d, Point3d>> DownCurve;

public:
	bsp_Frame2();
	~bsp_Frame2();

	void findLR(Mesh& mesh);
	void initUpDown(Mesh& mesh);

	void findfeatureLines(Mesh& mesh);
	void findfeatureVerts(Mesh& mesh);
};

