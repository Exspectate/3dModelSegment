#pragma once


#include "lty_STL.h"
typedef pair<int, int> PII;		//表示一个边

class bsp_part
{
public:
	vector<int> inVerts;		//选中点-int
	vector<int> inFacets;		//选中面片-int
	vector<PII> inbadEdge;		//坏边-PII
	vector<Point3d> featureVerts;	//关键点
	vector<vector<Point3d> > featureLines;		//特征线
	int color;					//颜色-部件标示


public:
	bsp_part();
	virtual ~bsp_part();
	virtual void findfeatureLines(Mesh & mesh);	//找特征线
	virtual void findfeatureVerts(Mesh & mesh); //找特征点
	void initSelectData(Mesh & mesh, int color);	//初始化

	void writeFeatureLinesToFile(string filePath);	//写特征线到文件中
	int findPartsPoint(Mesh &mesh, int color);
};


struct P_L {
	double l;
	Point3d p;
	P_L(double l, Point3d p) { this->l = l; this->p = p; }
};


bool comppl(const P_L & a, const P_L & b);