#pragma once


#include "lty_STL.h"
typedef pair<int, int> PII;		//��ʾһ����

class bsp_part
{
public:
	vector<int> inVerts;		//ѡ�е�-int
	vector<int> inFacets;		//ѡ����Ƭ-int
	vector<PII> inbadEdge;		//����-PII
	vector<Point3d> featureVerts;	//�ؼ���
	vector<vector<Point3d> > featureLines;		//������
	int color;					//��ɫ-������ʾ


public:
	bsp_part();
	virtual ~bsp_part();
	virtual void findfeatureLines(Mesh & mesh);	//��������
	virtual void findfeatureVerts(Mesh & mesh); //��������
	void initSelectData(Mesh & mesh, int color);	//��ʼ��

	void writeFeatureLinesToFile(string filePath);	//д�����ߵ��ļ���
	int findPartsPoint(Mesh &mesh, int color);
};


struct P_L {
	double l;
	Point3d p;
	P_L(double l, Point3d p) { this->l = l; this->p = p; }
};


bool comppl(const P_L & a, const P_L & b);